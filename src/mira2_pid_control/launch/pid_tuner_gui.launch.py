from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_id', default_value='28',
            description='ArUco marker ID to track'),

        DeclareLaunchArgument(
            'rtsp_url',
            default_value='rtsp://192.168.2.6:2000/image_rtsp',
            description='RTSP stream URL or local video file path'),

        Node(
            package='mira2_pid_control',
            executable='pid_tuner_gui.py',
            name='pid_tuner_gui',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_id': LaunchConfiguration('target_id'),
                'rtsp_url': LaunchConfiguration('rtsp_url'),
            }],
        ),
    ])
