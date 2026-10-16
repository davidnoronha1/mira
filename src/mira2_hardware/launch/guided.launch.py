"""Launch the GUIDED Pixhawk controller and its MAVLink plugins."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def guided(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    simulation = value('simulation') == 'true'
    source = value('pose_source') or {'vo': 'slam', 'gt': 'gt', 'none': 'optical_flow'}.get(value('source'), value('source'))
    if source not in ('slam', 'optical_flow', 'gt'):
        raise ValueError('pose_source must be slam, optical_flow or gt')
    if source == 'gt' and not simulation:
        raise ValueError('Ground-truth navigation requires simulation:=true')
    link = value('pixhawk_address') or value('goal_link') or ('udpin:0.0.0.0:14557' if simulation else '')
    if not link:
        raise ValueError('pixhawk_address is required on hardware')
    flow = source == 'optical_flow' or value('slam_optical_flow') == 'true'
    parameters = {
        'pixhawk_address': link, 'use_sim_time': simulation, 'simulation': simulation,
        'visualize': value('visualize') == 'true', 'auto_arm': value('auto_arm') == 'true',
        'external_navigation.source': {'slam': 'vo', 'optical_flow': 'none', 'gt': 'gt'}[source],
        'external_navigation.start_x': float(value('start_x')) if simulation else 0.0,
        'external_navigation.start_y': float(value('start_y')) if simulation else 0.0,
        'external_navigation.ekf_source_set': 2 if source == 'optical_flow' else 1,
        'external_navigation.flow_topic': '/vision/flow_velocity' if flow else '',
        'external_navigation.heading_topic': '/mira/perception/bottom/floor_heading' if source == 'optical_flow' else '',
    }
    return [Node(package='mira2_hardware', executable='pixhawk_controller',
                        namespace='mira', output='screen', parameters=[PathJoinSubstitution(
                            [FindPackageShare('mira2_hardware'), 'config', 'command_calibration.yaml']),
                            value('hardware_config'), parameters])]


def generate_launch_description():
    config = lambda package, name: PathJoinSubstitution([FindPackageShare(package), 'config', name])
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in {
            'simulation': 'false', 'source': 'vo', 'pose_source': '',
            'slam_optical_flow': 'false',
            'pixhawk_address': '', 'goal_link': '', 'auto_arm': 'false', 'visualize': 'false',
            'start_x': '0', 'start_y': '0',
        }.items()],
        DeclareLaunchArgument('hardware_config', default_value=config('mira2_hardware', 'guided.yaml')),
        OpaqueFunction(function=guided),
    ])
