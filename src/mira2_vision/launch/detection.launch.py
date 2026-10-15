"""One process per camera; launch selects every attached vision plugin."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from utils.parameters import string_list_argument


def launch_worker(context):
    config = LaunchConfiguration('config').perform(context)
    parameters = {}
    for name in ('detectors', 'consumers'):
        selection = LaunchConfiguration(name).perform(context).strip()
        if selection:
            parameters[name] = string_list_argument(selection)
    return [Node(package='mira2_vision', executable='mira2_vision',
                 name='perception', namespace='mira/perception/' + LaunchConfiguration('camera').perform(context),
                 output='screen', parameters=[config, parameters])]


def generate_launch_description():
    share = Path(get_package_share_directory('mira2_vision'))
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='front'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/front.yaml')),
        DeclareLaunchArgument('detectors', default_value='', description='Comma-separated plugins; blank uses config'),
        DeclareLaunchArgument('consumers', default_value='', description='Comma-separated plugins; [] disables all'),
        OpaqueFunction(function=launch_worker),
    ])
