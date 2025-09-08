from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mira2_dashboard',
            executable='mira2_dashboard_exe',  # For C++ packages
            name='mira2_dashboard_node'
        ),
    ])
