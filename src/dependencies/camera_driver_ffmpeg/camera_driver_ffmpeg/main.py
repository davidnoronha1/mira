#!/usr/bin/env python3


import subprocess
import glob
import rclpy
from rclpy.node import Node


class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_driver_ffmpeg_node')


        # Declare parameters with defaults matching your launch file
        self.declare_parameter('vendor_id', -1)
        self.declare_parameter('product_id', -1)
        self.declare_parameter('serial_no', '')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('frame_format', 'MJPEG')
        self.declare_parameter('framerate', 30)
        self.declare_parameter('port', 8554)
        self.declare_parameter('multicast_address', '239.255.0.1')


        # Get parameter values
        vendor = self.get_parameter('vendor_id').value
        product = self.get_parameter('product_id').value
        serial = self.get_parameter('serial_no').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        fmt = self.get_parameter('frame_format').value
        framerate = self.get_parameter('framerate').value
        port = self.get_parameter('port').value
        multicast = self.get_parameter('multicast_address').value


        # Find webcam device
        dev = self.find_webcam(vendor, product, serial)
        if not dev:
            raise RuntimeError("Webcam not found")
        self.get_logger().info(f"Found webcam: {dev}")


        # Print stream configuration details
        self.get_logger().info("=" * 60)
        self.get_logger().info("Stream Configuration:")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Device:             {dev}")
        self.get_logger().info(f"Image Width:        {width} px")
        self.get_logger().info(f"Image Height:       {height} px")
        self.get_logger().info(f"Frame Format:       {fmt}")
        self.get_logger().info(f"Framerate:          {framerate} fps")
        self.get_logger().info(f"Multicast Address:  {multicast}")
        self.get_logger().info(f"Port:               {port}")
        self.get_logger().info(f"Stream URL:         udp://{multicast}:{port}")
        if vendor is not None and vendor != -1:
            self.get_logger().info(f"Vendor ID:          0x{vendor:04x}")
        if product is not None and product != -1:
            self.get_logger().info(f"Product ID:         0x{product:04x}")
        if serial:
            self.get_logger().info(f"Serial Number:      {serial}")
        self.get_logger().info("=" * 60)


        # Build FFmpeg command
        cmd = [
            'ffmpeg',
            '-f', 'v4l2',
            '-input_format', fmt.lower(),
            '-video_size', f"{width}x{height}",
            '-framerate', str(framerate),
            '-i', dev,
            '-c:v', 'copy',
            '-f', 'mjpeg',
            '-fflags', 'nobuffer',
            '-loglevel', 'info',
            f'udp://{multicast}:{port}?pkt_size=1316'
        ]


        # Run FFmpeg
        self.get_logger().info(f"Starting FFmpeg with command: {' '.join(cmd)}")
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            self.get_logger().info("Interrupted by user, stopping ffmpeg...")
        except Exception as e:
            self.get_logger().error(f"Failed to start ffmpeg: {e}")


    def find_webcam(self, vendor=None, product=None, serial=None):
        """
        Find a webcam device matching vendor, product, and serial.
        """
        devices = sorted(glob.glob("/dev/video*"))


        vendor = None if vendor == -1 else vendor
        product = None if product == -1 else product


        vendor = f"{vendor:04x}" if vendor is not None else None
        product = f"{product:04x}" if product is not None else None


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



def main():
    rclpy.init()
    try:
        node = CameraStreamer()
    except RuntimeError as e:
        print("Encountered an error:", e)
        rclpy.shutdown()
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
