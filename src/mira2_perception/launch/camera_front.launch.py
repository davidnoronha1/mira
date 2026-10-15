import os
from ament_index_python.packages import get_package_share_directory
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
            remappings=[('/mira/sensors/camera/front/image', '/mira/sensors/camera/front/image_raw')],
            parameters=[{
                'port': 2001,
                'usb_port': usb_port,
                'camera_frame_id': 'camera_front_optical',
                'ros_topic': '/mira/sensors/camera/front'
            }, os.path.join(get_package_share_directory("mira2_perception"), "config", "capture.yaml")]
        ),
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5 ; gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:2001/image_rtsp ! fakesink'],
            cwd='/home'
        )
    ])
