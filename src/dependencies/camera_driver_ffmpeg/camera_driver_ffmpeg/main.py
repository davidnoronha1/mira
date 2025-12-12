#!/usr/bin/env python3

import subprocess
import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import re
import sys
import selectors

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_driver_ffmpeg_node')

        self.declare_parameter('device_path', '')
        self.declare_parameter('vendor_id', -1)
        self.declare_parameter('product_id', -1)
        self.declare_parameter('serial_no', '')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('frame_format', 'MJPEG')
        self.declare_parameter('framerate', 30)
        self.declare_parameter('topic', '/camera/image/compressed')

        vendor = self.get_parameter('vendor_id').value
        product = self.get_parameter('product_id').value
        serial = self.get_parameter('serial_no').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        fmt = self.get_parameter('frame_format').value
        framerate = self.get_parameter('framerate').value
        topic = self.get_parameter('topic').value
        dev = self.get_parameter('device_path').value

        if dev == '':
            dev = self.find_webcam(vendor, product, serial)
            if not dev:
                raise RuntimeError("Webcam not found")

        self.pub = self.create_publisher(CompressedImage, topic, 10)

        self.ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'v4l2',
            '-input_format', fmt.lower(),
            '-video_size', f"{width}x{height}",
            '-framerate', str(framerate),
            '-i', dev,
            '-c:v', 'copy',
            '-f', 'mjpeg',
            '-'
        ]

        self.get_logger().info("Starting FFmpeg...")
        self.proc = subprocess.Popen(
            self.ffmpeg_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,     # ← SHOW STDERR
            bufsize=0
        )

        # Non-blocking stderr reader
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.proc.stderr, selectors.EVENT_READ)

        # JPEG extraction regex
        self.boundary_regex = re.compile(b'\xff\xd8.*?\xff\xd9', re.DOTALL)

        self.read_loop()

    def find_webcam(self, vendor=None, product=None, serial=None):
        devices = sorted(glob.glob("/dev/video*"))
        vendor = None if vendor == -1 else f"{vendor:04x}"
        product = None if product == -1 else f"{product:04x}"

        for dev in devices:
            try:
                info = subprocess.check_output(
                    ["udevadm", "info", "--query=all", "--name", dev],
                    text=True
                )
            except subprocess.CalledProcessError:
                continue

            if vendor and f"ID_VENDOR_ID={vendor}" not in info:
                continue
            if product and f"ID_MODEL_ID={product}" not in info:
                continue
            if serial and f"ID_USB_SERIAL_SHORT={serial}" not in info:
                continue

            return dev
        return None

    def read_loop(self):
        buffer = b""

        while rclpy.ok():
            # ---- READ STDOUT FRAME CHUNKS ----
            chunk = self.proc.stdout.read(4096)
            if chunk:
                buffer += chunk

                while True:
                    match = self.boundary_regex.search(buffer)
                    if not match:
                        break

                    jpeg = match.group(0)
                    buffer = buffer[match.end():]

                    msg = CompressedImage()
                    msg.format = "jpeg"
                    msg.data = jpeg
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera"

                    self.pub.publish(msg)

            # ---- READ STDERR NON-BLOCKING ----
            for key, _ in self.selector.select(timeout=0):
                line = key.fileobj.readline()
                if not line:
                    continue
                sys.stderr.write(line.decode('utf-8', errors='ignore'))
                sys.stderr.flush()

            # Let ROS process callbacks
            rclpy.spin_once(self, timeout_sec=0)

        self.get_logger().error("FFmpeg stopped producing frames.")

def main():
    rclpy.init()
    node = None
    try:
        node = CameraStreamer()
    except RuntimeError as e:
        print(f"Error: {e}")
        rclpy.shutdown()
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

