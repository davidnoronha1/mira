#!/usr/bin/env python3
"""
Camera Device Information Gathering Script
Collects information from lsusb, v4l2-ctl, libcamera (cam), and gstreamer
"""

import subprocess
import re
import sys

# ANSI color codes
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

def run_command(cmd, shell=False):
    """Run a command and return output, handling errors gracefully"""
    try:
        if shell:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
        else:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        return "", "Command timed out", -1
    except FileNotFoundError:
        return "", "Command not found", -1
    except Exception as e:
        return "", str(e), -1

def print_header(title, char='═'):
    """Print a bold colored header"""
    print(f"\n{Colors.BOLD}{Colors.CYAN}{char * 80}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.YELLOW}{title}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{char * 80}{Colors.END}")

def print_subheader(title):
    """Print a subheader"""
    print(f"\n{Colors.BOLD}{Colors.GREEN}┌{'─' * 78}┐{Colors.END}")
    print(f"{Colors.BOLD}{Colors.GREEN}│{Colors.END} {Colors.BOLD}{title}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.GREEN}└{'─' * 78}┘{Colors.END}")

def print_table_row(label, value, width=78):
    """Print a formatted table row"""
    label_colored = f"{Colors.BOLD}{Colors.BLUE}{label}{Colors.END}"
    # Calculate actual display width (ANSI codes don't count)
    label_display = label
    padding = width - len(label_display) - len(str(value))
    print(f"  {label_colored}: {' ' * padding}{value}")

def get_usb_device_details(bus, device):
    """Get detailed information for a specific USB device including serial number"""
    stdout, stderr, returncode = run_command(['lsusb', '-v', '-s', f'{bus}:{device}'])

    if returncode != 0:
        return None

    serial = None
    for line in stdout.split('\n'):
        if 'iSerial' in line:
            parts = line.split()
            if len(parts) >= 3:
                serial = ' '.join(parts[2:])
            break

    return serial

def list_usb_cameras():
    """List only cameras from lsusb with detailed info"""
    print_header("1. USB CAMERAS")

    stdout, stderr, returncode = run_command(['lsusb'])

    if returncode != 0:
        print(f"{Colors.RED}Error running lsusb: {stderr}{Colors.END}")
        return

    if not stdout:
        print("No USB devices found or lsusb not available")
        return

    lines = stdout.strip().split('\n')
    camera_keywords = ['camera', 'webcam', 'video', 'imaging']

    camera_count = 0
    for line in lines:
        is_camera = any(keyword in line.lower() for keyword in camera_keywords)
        if is_camera:
            camera_count += 1
            match = re.search(r'Bus (\d+) Device (\d+): ID ([0-9a-f]{4}):([0-9a-f]{4})\s+(.*)', line)
            if match:
                bus = match.group(1)
                device = match.group(2)
                vendor_id = match.group(3)
                product_id = match.group(4)
                name = match.group(5).strip()

                serial = get_usb_device_details(bus, device)

                print_subheader(f"Camera {camera_count}")
                print_table_row("Name", name)
                print_table_row("Vendor ID", vendor_id)
                print_table_row("Product ID", product_id)
                print_table_row("Serial Number", serial if serial else 'N/A')

    if camera_count == 0:
        print(f"{Colors.YELLOW}No camera devices detected in USB devices{Colors.END}")

def list_v4l2_devices():
    """List all V4L2 devices with camera names"""
    print_header("2. V4L2 DEVICES")

    stdout, stderr, returncode = run_command(['v4l2-ctl', '--list-devices'])

    if returncode != 0:
        print(f"{Colors.RED}Error running v4l2-ctl: {stderr}{Colors.END}")
        return [], {}

    if not stdout:
        print("No V4L2 devices found")
        return [], {}

    device_map = {}
    current_camera = None

    for line in stdout.split('\n'):
        line = line.rstrip()
        if not line:
            continue

        if not line.startswith((' ', '\t')):
            current_camera = re.sub(r'\s*\([^)]*\)\s*:?\s*$', '', line).strip()
        elif line.strip().startswith('/dev/video'):
            device_path = line.strip()
            if current_camera:
                device_map[device_path] = current_camera

    # Print as table
    print(f"\n{Colors.BOLD}{'Device':<20} {'Camera Name'}{Colors.END}")
    print(f"{Colors.CYAN}{'─' * 80}{Colors.END}")
    for device_path in sorted(device_map.keys()):
        print(f"{Colors.GREEN}{device_path:<20}{Colors.END} {device_map[device_path]}")

    return list(device_map.keys()), device_map

def list_libcamera_devices():
    """List all libcamera devices"""
    print_header("3. LIBCAMERA DEVICES")

    stdout, stderr, returncode = run_command(['cam', '-l'])

    if returncode != 0:
        print(f"{Colors.RED}Error running cam -l: {stderr}{Colors.END}")
        if "not found" in stderr.lower():
            print(f"{Colors.YELLOW}libcamera-apps may not be installed. Install with: sudo apt install libcamera-apps{Colors.END}")
        return []

    if not stdout:
        print("No libcamera devices found")
        return []

    print(stdout)

    camera_indices = []
    for line in stdout.split('\n'):
        match = re.search(r'^\s*(\d+)\s*:', line)
        if match:
            camera_indices.append(match.group(1))

    return camera_indices

def is_valid_video_device(device_path):
    """Check if a device is an actual camera with video capabilities"""
    stdout, stderr, returncode = run_command(['v4l2-ctl', '-d', device_path, '--list-formats-ext'])

    if returncode != 0:
        return False

    if 'video capture' in stdout.lower() or 'pixelformat' in stdout.lower():
        return True

    return False

def format_v4l2_output(output):
    """Add color formatting to v4l2-ctl output"""
    lines = output.split('\n')
    formatted = []

    for line in lines:
        # Highlight section headers
        if line.endswith(':') and not line.startswith(' '):
            formatted.append(f"{Colors.BOLD}{Colors.CYAN}{line}{Colors.END}")
        # Highlight property names
        elif ':' in line and line.startswith(' ' * 8):
            parts = line.split(':', 1)
            if len(parts) == 2:
                formatted.append(f"{Colors.BLUE}{parts[0]}{Colors.END}:{parts[1]}")
            else:
                formatted.append(line)
        else:
            formatted.append(line)

    return '\n'.join(formatted)

def get_v4l2_device_info(device_paths, device_map):
    """Get detailed info for each V4L2 device"""
    print_header("4. DETAILED V4L2 DEVICE INFO")

    if not device_paths:
        print("No V4L2 devices to query")
        return

    for device in sorted(device_paths):
        camera_name = device_map.get(device, "Unknown Camera")

        print_subheader(f"Device: {device} | Camera: {camera_name}")

        if not is_valid_video_device(device):
            print(f"{Colors.YELLOW}Skipping {device} - not a video capture device or no video feeds available{Colors.END}")
            continue

        stdout, stderr, returncode = run_command(['v4l2-ctl', '--all', '-d', device])

        if returncode != 0:
            print(f"{Colors.RED}Error getting info for {device}: {stderr}{Colors.END}")
            continue

        print(format_v4l2_output(stdout))

def get_libcamera_info(camera_indices):
    """Get detailed info for each libcamera device"""
    print_header("5. DETAILED LIBCAMERA INFO")

    if not camera_indices:
        print("No libcamera devices to query")
        return

    for index in camera_indices:
        print_subheader(f"Camera Index: {index}")

        stdout, stderr, returncode = run_command(['cam', '-I', '-c', index])

        if returncode != 0:
            print(f"{Colors.RED}Error getting info for camera {index}: {stderr}{Colors.END}")
            continue

        print(stdout)

def format_gstreamer_output(output):
    """Format gstreamer output with colors"""
    lines = output.split('\n')
    formatted = []

    for line in lines:
        if line.strip().startswith('name'):
            formatted.append(f"{Colors.BOLD}{Colors.GREEN}{line}{Colors.END}")
        elif line.strip().startswith('class'):
            formatted.append(f"{Colors.BOLD}{Colors.CYAN}{line}{Colors.END}")
        elif line.strip().startswith('caps'):
            formatted.append(f"{Colors.BOLD}{Colors.BLUE}{line}{Colors.END}")
        elif 'gst-launch' in line:
            formatted.append(f"{Colors.YELLOW}{line}{Colors.END}")
        elif line.strip() == 'Device found:':
            formatted.append(f"\n{Colors.BOLD}{Colors.CYAN}{'─' * 80}{Colors.END}")
            formatted.append(f"{Colors.BOLD}{line}{Colors.END}")
        else:
            formatted.append(line)

    return '\n'.join(formatted)

def get_gstreamer_info():
    """Get GStreamer video source information"""
    print_header("6. GSTREAMER VIDEO SOURCES")

    stdout, stderr, returncode = run_command(['gst-device-monitor-1.0', 'Video/Source'])

    if returncode != 0:
        print(f"{Colors.RED}Error running gst-device-monitor-1.0: {stderr}{Colors.END}")
        if "not found" in stderr.lower():
            print(f"{Colors.YELLOW}GStreamer tools may not be installed. Install with: sudo apt install gstreamer1.0-tools{Colors.END}")
        return

    if not stdout:
        print("No GStreamer video sources found")
        return

    print(format_gstreamer_output(stdout))

def main():
    print(f"{Colors.BOLD}{Colors.CYAN}{'═' * 80}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.YELLOW}CAMERA DEVICE INFORMATION GATHERING{Colors.END}")
    timestamp = subprocess.run(['date'], capture_output=True, text=True).stdout.strip()
    print(f"{Colors.BOLD}Timestamp: {Colors.GREEN}{timestamp}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'═' * 80}{Colors.END}")

    list_usb_cameras()

    result = list_v4l2_devices()
    if result:
        v4l2_devices, device_map = result
    else:
        v4l2_devices, device_map = [], {}

    libcamera_indices = list_libcamera_devices()

    get_v4l2_device_info(v4l2_devices, device_map)

    get_libcamera_info(libcamera_indices)

    get_gstreamer_info()

    print(f"\n{Colors.BOLD}{Colors.CYAN}{'═' * 80}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.GREEN}✓ SCAN COMPLETE{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'═' * 80}{Colors.END}\n")

if __name__ == "__main__":
    main()
