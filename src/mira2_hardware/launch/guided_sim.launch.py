"""Run the GUIDED Pixhawk controller against an independently running Gazebo/SITL.

Gazebo and ros_gz_bridge belong to mira_sim. This launch only selects ROS
navigation forwarding, simulation time, and the localhost MAVLink endpoint.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_guided(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    arguments = {key: value(key) for key in DEFAULTS}
    arguments.update(
        simulation='true',
        pixhawk_address=f'udpin:{value("bind_ip")}:14557',
        hardware_config=LaunchConfiguration('hardware_config'),
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mira2_hardware'), 'launch', 'guided.launch.py'])),
        launch_arguments=arguments.items())]


DEFAULTS = {
    'bind_ip': '127.0.0.1', 'auto_arm': 'false', 'visualize': 'false',
    'pose_source': 'slam', 'slam_optical_flow': 'false',
    'start_x': '0', 'start_y': '0',
}


def generate_launch_description():
    return LaunchDescription([
        # Match mira_sim's bridge transport; no router/container is required.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        *[DeclareLaunchArgument(key, default_value=value) for key, value in DEFAULTS.items()],
        DeclareLaunchArgument('hardware_config', default_value=PathJoinSubstitution(
            [FindPackageShare('mira2_hardware'), 'config', 'guided.yaml'])),
        OpaqueFunction(function=include_guided),
    ])
