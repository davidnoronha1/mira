"""Compose camera-owned navigation with the hardware GUIDED goal bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def goal_bridge(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    simulation = value('simulation') == 'true'
    link = value('goal_link') or ('udpin:0.0.0.0:14557' if simulation else '')
    if not link:
        return []  # Hardware goal transport must be configured explicitly.
    return [Node(package='mira2_hardware', executable='goal_bridge', output='screen',
                 parameters=[{'use_sim_time': simulation}],
                 arguments=['--mavlink', link, '--simulation', value('simulation'),
                            '--visualize', value('visualize'), '--auto-arm', value('auto_arm')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('goal_link', default_value=''),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('visualize', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mira2_vision'), 'launch', 'navigation.launch.py'])),
            launch_arguments={'simulation': LaunchConfiguration('simulation')}.items()),
        OpaqueFunction(function=goal_bridge),
    ])
