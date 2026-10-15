from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def launch_mission(context):
    mission = LaunchConfiguration('mission').perform(context)
    executables = {'gate': 'gate_navigator_exe', 'docking': 'phase2_dock_controls.py',
                   'bucket': 'phase2.py', 'pipeline': 'pipeline_controls.py'}
    if mission not in executables:
        raise ValueError('Unknown mission: ' + mission)
    config = Path(get_package_share_directory('mira2_path_planning')) / 'config' / (mission + '.yaml')
    return [Node(package='mira2_path_planning', executable=executables[mission],
                 parameters=[str(config)], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mission', default_value='gate'),
                              OpaqueFunction(function=launch_mission)])
