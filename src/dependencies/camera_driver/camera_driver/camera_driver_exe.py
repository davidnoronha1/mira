#!/usr/bin/env python3

import os
os.environ['GST_DEBUG'] = '2'
import subprocess
import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import threading
import yaml

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_gstreamer_node')
        
        # Declare parameters with defaults
        self.declare_parameter('device_path', '')
        self.declare_parameter('vendor_id', -1)
        self.declare_parameter('product_id', -1)
        self.declare_parameter('serial_no', '')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('frame_format', 'MJPEG')
        self.declare_parameter('framerate', 30)
        self.declare_parameter('topic', '/camera/image/compressed')
        self.declare_parameter('calibration_file', '')
        
        # Get parameter values
        vendor = self.get_parameter('vendor_id').value
        product = self.get_parameter('product_id').value
        serial = self.get_parameter('serial_no').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        fmt = self.get_parameter('frame_format').value
        self.framerate = self.get_parameter('framerate').value
        self.topic = self.get_parameter('topic').value
        calibration_file = self.get_parameter('calibration_file').value
        dev = self.get_parameter('device_path').value
        
        # Load calibration and camera parameters from YAML
        self.camera_params = ''
        self.camera_info = None
        if calibration_file:
            self.load_calibration_file(calibration_file)
        
        # Find webcam device
        if dev == '':
            dev = self.find_webcam(vendor, product, serial)
            if not dev:
                raise RuntimeError("Webcam not found")
        self.device = dev
        self.get_logger().info(f"Found webcam: {self.device}")
        
        # Print configuration details
        self.get_logger().info("=" * 60)
        self.get_logger().info("Camera Publisher Configuration:")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Device:             {self.device}")
        self.get_logger().info(f"Image Width:        {self.width} px")
        self.get_logger().info(f"Image Height:       {self.height} px")
        self.get_logger().info(f"Frame Format:       {fmt}")
        self.get_logger().info(f"Framerate:          {self.framerate} fps")
        self.get_logger().info(f"Topic:              {self.topic}")
        if vendor is not None and vendor != -1:
            self.get_logger().info(f"Vendor ID:          0x{vendor:04x}")
        if product is not None and product != -1:
            self.get_logger().info(f"Product ID:         0x{product:04x}")
        if serial:
            self.get_logger().info(f"Serial Number:      {serial}")
        if calibration_file:
            self.get_logger().info(f"Calibration File:   {calibration_file}")
        if self.camera_params:
            self.get_logger().info(f"Camera Parameters:  {self.camera_params}")
        if self.camera_info:
            self.get_logger().info(f"Camera Info:        Loaded")
        self.get_logger().info("=" * 60)
        
        # Create publishers
        self.pub = self.create_publisher(CompressedImage, self.topic, 10)
        if self.camera_info:
            camera_info_topic = self.topic.replace('/compressed', '/camera_info')
            self.info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Build GStreamer pipeline
        # Add extra_controls if camera_params is specified
        extra_controls = f' extra-controls="{self.camera_params}"' if self.camera_params else ''
        
        pipeline_str = (
            f'v4l2src device={self.device}{extra_controls} ! '
            f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
            f'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )
        
        self.get_logger().info(f"GStreamer Pipeline: {pipeline_str}")
        
        # Create pipeline
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(f"Failed to create GStreamer pipeline: {e}")
            raise RuntimeError(f"GStreamer pipeline creation failed: {e}")
        
        # Get appsink element
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        
        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Unable to set pipeline to playing state")
            raise RuntimeError("Failed to start GStreamer pipeline")
        
        self.get_logger().info("GStreamer pipeline started successfully")
    
    def load_calibration_file(self, filepath):
        """Load camera parameters and calibration from YAML file"""
        if not os.path.exists(filepath):
            self.get_logger().warn(f"Calibration file not found: {filepath}")
            return
        
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            if not data:
                self.get_logger().warn(f"Empty calibration file: {filepath}")
                return
            
            # Load camera_params if present
            if 'camera_params' in data:
                self.camera_params = data['camera_params']
                self.get_logger().info(f"Loaded camera parameters from file")
            
            # Load camera calibration if present
            if 'camera_matrix' in data or 'camera_info' in data:
                self.camera_info = CameraInfo()
                
                # Try to load from camera_info structure first
                if 'camera_info' in data:
                    info = data['camera_info']
                    if 'K' in info:
                        self.camera_info.k = info['K']
                    if 'D' in info:
                        self.camera_info.d = info['D']
                    if 'R' in info:
                        self.camera_info.r = info['R']
                    if 'P' in info:
                        self.camera_info.p = info['P']
                    if 'distortion_model' in info:
                        self.camera_info.distortion_model = info['distortion_model']
                    if 'width' in info:
                        self.camera_info.width = info['width']
                    if 'height' in info:
                        self.camera_info.height = info['height']
                
                # Or load from individual fields
                else:
                    if 'camera_matrix' in data:
                        self.camera_info.k = data['camera_matrix']
                    if 'distortion_coefficients' in data:
                        self.camera_info.d = data['distortion_coefficients']
                    if 'rectification_matrix' in data:
                        self.camera_info.r = data['rectification_matrix']
                    if 'projection_matrix' in data:
                        self.camera_info.p = data['projection_matrix']
                    if 'distortion_model' in data:
                        self.camera_info.distortion_model = data['distortion_model']
                    else:
                        self.camera_info.distortion_model = 'plumb_bob'
                
                # Set dimensions if not already set
                if self.camera_info.width == 0:
                    self.camera_info.width = self.width
                if self.camera_info.height == 0:
                    self.camera_info.height = self.height
                
                self.get_logger().info("Loaded camera calibration from file")
        
        except yaml.YAMLError as e:
            self.get_logger().warn(f"Failed to parse YAML file: {e}")
        except Exception as e:
            self.get_logger().warn(f"Failed to load calibration file: {e}")
    
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
    
    def on_new_sample(self, sink):
        """Callback when new frame is available"""
        sample = sink.emit('pull-sample')
        if sample:
            buf = sample.get_buffer()
            
            # Extract buffer data
            success, map_info = buf.map(Gst.MapFlags.READ)
            if success:
                timestamp = self.get_clock().now().to_msg()
                
                # Create CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = timestamp
                msg.header.frame_id = "camera"
                msg.format = "jpeg"
                msg.data = bytes(map_info.data)
                
                # Publish message
                self.pub.publish(msg)
                
                # Publish camera info if available
                if self.camera_info:
                    self.camera_info.header.stamp = timestamp
                    self.camera_info.header.frame_id = "camera"
                    self.info_pub.publish(self.camera_info)
                
                # Unmap buffer
                buf.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def shutdown(self):
        """Clean shutdown of pipeline"""
        self.get_logger().info("Shutting down GStreamer pipeline")
        self.pipeline.set_state(Gst.State.NULL)

def main():
    rclpy.init()
    node = None
    try:
        node = CameraPublisher()
        
        # Spin in a separate thread to allow GStreamer callbacks
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        
        # Keep main thread alive
        spin_thread.join()
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Interrupted by user, stopping...")
    except RuntimeError as e:
        print(f"Encountered an error: {e}")
    except Exception as e:
        if node:
            node.get_logger().error(f"Error: {e}")
        else:
            print(f"Error: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
