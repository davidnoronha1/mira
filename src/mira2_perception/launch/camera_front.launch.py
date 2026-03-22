import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    machine = os.environ.get('MACHINE_NAME', 'ORIN')
    # Front camera USB paths
    usb_path = "usb-3610000.usb-2.1" if machine == 'RPI4' else "usb-xhci-hcd.1-1"

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_front',
            parameters=[{
                'video_device': f'/dev/v4l/by-path/platform-{usb_path}-video-index0',
                'camera_frame_id': 'camera_front_link'
            }]
        )
    ])