"""Run the single visible-object navigation demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_class', default_value='dock'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='mira2_actions', executable='mira2_actions_exe', output='screen',
             parameters=[{'target_class': ParameterValue(LaunchConfiguration('target_class'), value_type=str),
                          'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
