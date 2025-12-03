from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_boundingbox',
            executable='vision_boundingbox_exe',  # For C++ packages
            name='vision_boundingbox_node'
        ),
    ])
