import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_launch_dir = os.path.join(get_package_share_directory('mira2_perception'), 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_launch_dir, 'camera_front.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_launch_dir, 'camera_bottom.launch.py')))
    ])