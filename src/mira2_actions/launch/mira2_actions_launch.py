from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mira2_actions',
            executable='mira2_actions_exe',  # For C++ packages
            name='mira2_actions_node'
        ),
    ])
