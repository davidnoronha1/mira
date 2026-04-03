from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('mira2_pid_control')
    config_file = os.path.join(pkg_share, 'config', 'video_submission.yaml')
    
    return LaunchDescription([
        # Video submission controller with parameters
        Node(
            package='mira2_pid_control',
            executable='video_hardcode_exe',
            name='video_submission_controller',
            output='screen',
            parameters=[config_file] if os.path.exists(config_file) else [],
            emulate_tty=True,
        ),
        
        # Key publisher for manual control
        Node(
            package='mira2_pid_control',
            executable='key_pub.py',
            name='key_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ])
