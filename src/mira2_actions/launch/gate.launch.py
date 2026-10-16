"""Acquire a frontal gate, align before it, and navigate through it.

This routine only runs the behavior tree. Start recording/SLAM tools separately.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_defaults = {'scan_radius': '0.0', 'scan_sweep': '360.0', 'scan_step': '30.0', 'scan_depth': '1.0'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in scan_defaults.items()],
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='mira2_actions', executable='mira2_actions_exe', output='screen',
             parameters=[{
                 **{k: ParameterValue(LaunchConfiguration(k), value_type=float) for k in scan_defaults},
                 'target_class': 'gate',
                 'tree_xml': PathJoinSubstitution(
                     [FindPackageShare('mira2_actions'), 'config', 'gate.xml']),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
