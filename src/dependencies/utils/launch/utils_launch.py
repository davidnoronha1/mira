from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='utils',
            executable='utils_exe',  # For C++ packages
            name='utils_node'
        ),
    ])
