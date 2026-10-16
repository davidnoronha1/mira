"""Start one Pixhawk connection with the selected controller plugins."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from utils.parameters import string_list_argument


def controller(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = lambda name: PathJoinSubstitution([FindPackageShare('mira2_hardware'), 'config', name])
    return [Node(
        package='mira2_hardware', executable='pixhawk_controller', namespace='mira', output='screen',
        parameters=[config('master.yaml'), config('command_calibration.yaml'), {
            'pixhawk_address': value('pixhawk_address'),
            'plugins': string_list_argument(value('plugins')),
        }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pixhawk_address', default_value='/dev/Pixhawk'),
        DeclareLaunchArgument('plugins', default_value='rc,telemetry'),
        OpaqueFunction(function=controller),
    ])
