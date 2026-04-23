from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='mira2_pid_control',
            executable='lateral_tuning_exe',
            name='lateral_tuner',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='mira2_pid_control',
            executable='key_pub.py',
            name='key_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ])
