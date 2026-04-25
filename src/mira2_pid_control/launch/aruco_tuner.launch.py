from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'target_id', default_value='0',
            description='ArUco marker ID to track'),

        DeclareLaunchArgument(
            'axis', default_value='lateral',
            description='Axis to tune: lateral or forward'),

        DeclareLaunchArgument(
            'rtsp_url', default_value='rtsp://192.168.2.6:2000/image_rtsp',
            description='Video source URL (RTSP stream or file path)'),

        Node(
            package='mira2_perception',
            executable='aruco_tracker.py',
            name='aruco_tracker',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_id': LaunchConfiguration('target_id'),
                'rtsp_url': LaunchConfiguration('rtsp_url'),
            }],
        ),

        Node(
            package='mira2_pid_control',
            executable='lateral_tuning_exe',
            name='lateral_tuner',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('axis'), 'lateral')),
        ),

        Node(
            package='mira2_pid_control',
            executable='forward_tuning_exe',
            name='forward_tuner',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('axis'), 'forward')),
        ),

        Node(
            package='mira2_pid_control',
            executable='key_pub.py',
            name='key_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ])
