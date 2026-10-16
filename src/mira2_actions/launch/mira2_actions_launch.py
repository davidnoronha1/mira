"""Run the single visible-object navigation demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_defaults = {'scan_radius': '0.0', 'scan_sweep': '360.0', 'scan_step': '30.0', 'scan_depth': '1.0'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in scan_defaults.items()],
        DeclareLaunchArgument('target_class', default_value='dock'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('tree_xml', default_value=PathJoinSubstitution(
            [FindPackageShare('mira2_actions'), 'config', 'demo.xml'])),
        Node(package='mira2_actions', executable='mira2_actions_exe', output='screen',
             parameters=[{**{k: ParameterValue(LaunchConfiguration(k), value_type=float) for k in scan_defaults},
                          'tree_xml': LaunchConfiguration('tree_xml'),
                          'target_class': ParameterValue(LaunchConfiguration('target_class'), value_type=str),
                          'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
