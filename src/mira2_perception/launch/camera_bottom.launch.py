import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    machine = os.environ.get('MACHINE_NAME', 'ORIN')
    # Bottom camera USB paths
    usb_port = "usb-3610000.usb-2.4" if machine == 'RPI4' else "usb-3610000.usb-2.4"
    
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_driver_exe',
            name='camera_bottom_driver',
            output='screen',
            parameters=[{
                'vendor_id': 0x05a3,
                'product_id': 0x9420,
                'serial_no': 'SN0001',
                'image_width': 1280,
                'image_height': 720,
                'frame_format': 'MJPEG',
                'framerate': 30,
                'port': 2000,
                'usb_port': usb_port,
                'camera_frame_id': 'camera_bottom',
                'camera_info_url': 'package:///mira2_perception/config/bottomcam.ini'
            }]
        ),
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5 ; gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:2000/image_rtsp ! fakesink'],
            cwd='/home'
        )
    ])
