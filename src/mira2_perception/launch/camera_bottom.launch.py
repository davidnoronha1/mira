import os
from ament_index_python.packages import get_package_share_directory
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
            remappings=[('/mira/sensors/camera/bottom/image', '/mira/sensors/camera/bottom/image_raw')],
            parameters=[{
                'vendor_id': 0x05a3,
                'product_id': 0x9420,
                'serial_no': 'SN0001',
                'port': 2000,
                'usb_port': usb_port,
                'camera_frame_id': 'camera_bottom_optical',
                'ros_topic': '/mira/sensors/camera/bottom',
                'camera_info_url': 'package://mira2_perception/config/bottomcam.ini'
            }, os.path.join(get_package_share_directory("mira2_perception"), "config", "capture.yaml")]
        ),
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5 ; gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:2000/image_rtsp ! fakesink'],
            cwd='/home'
        )
    ])
