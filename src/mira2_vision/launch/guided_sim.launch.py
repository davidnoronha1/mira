"""Run guided perception and odometry against an independently running Gazebo/SITL.

Gazebo and ros_gz_bridge belong to mira_sim. This launch only selects ROS
sensor inputs and simulation time for the simulated cameras.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_navigation(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    arguments = {key: value(key) for key in DEFAULTS}
    arguments.update(
        simulation='true',
        config=LaunchConfiguration('config'),
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mira2_vision'), 'launch', 'navigation.launch.py'])),
        launch_arguments=arguments.items())]


DEFAULTS = {
    'rgb_topic': '/vision/image', 'depth_topic': '/vision/depth',
    'camera_info_topic': '/vision/camera_info', 'imu_topic': '/vision/imu',
    'pose_source': 'slam', 'slam_algorithm': 'cuvslam',
    'slam_optical_flow': 'false', 'slam_imu': 'false',
    'flow_scene_config': '', 'optical_flow_floor_depth': '',
    'detectors': 'gazebo_boxes', 'yolo_device': '', 'yolo_model': '',
}


def generate_launch_description():
    return LaunchDescription([
        # Match mira_sim's bridge transport; no router/container is required.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        *[DeclareLaunchArgument(key, default_value=value) for key, value in DEFAULTS.items()],
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [FindPackageShare('mira2_vision'), 'config', 'guided_front.yaml'])),
        OpaqueFunction(function=include_navigation),
    ])
