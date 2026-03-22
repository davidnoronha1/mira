import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    machine = os.environ.get('MACHINE_NAME', 'ORIN')
    # Bottom camera USB paths (Assuming different ports)
    usb_path = "usb-3610000.usb-2.2" if machine == 'RPI4' else "usb-xhci-hcd.1-2"

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_bottom',
            parameters=[{
                'video_device': f'/dev/v4l/by-path/platform-{usb_path}-video-index0',
                'camera_frame_id': 'camera_bottom_link'
            }]
        )
    ])