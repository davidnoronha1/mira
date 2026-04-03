import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    machine = os.environ.get('MACHINE_NAME', 'ORIN')
    # Front camera USB paths
    usb_port = "usb-3610000.usb-2.1" if machine == 'RPI4' else "usb-xhci-hcd.1-1"
    
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_driver_exe',
            name='camera_front_driver',
            output='screen',
            parameters=[{
                'image_width': 1280,
                'image_height': 720,
                'frame_format': 'MJPEG',
                'framerate': 30,
                'port': 2001,
                'usb_port': usb_port,
                'camera_frame_id': 'camera_front'
            }]
        ),
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5 ; gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:2001/image_rtsp ! fakesink'],
            cwd='/home'
        )
    ])