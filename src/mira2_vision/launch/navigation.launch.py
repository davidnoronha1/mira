"""ROS-only perception, optical flow and native cuVSLAM/RTAB-Map odometry."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from utils.parameters import string_list_argument
from mira2_vision.launch_config import NAVIGATION_ARGUMENTS, pose_source, select_backend


def camera_topics(value, simulation):
    defaults = ('/vision/image', '/vision/depth', '/vision/camera_info') if simulation else (
        '/mira/sensors/camera/front/image_raw', '/mira/sensors/camera/front/depth/image_raw',
        '/mira/sensors/camera/front/camera_info')
    return [value(name) or default for name, default in zip(
        ('rgb_topic', 'depth_topic', 'camera_info_topic'), defaults)]


def perception(value, simulation, topics):
    rgb, depth, info = topics
    parameters = {
        'simulation': simulation, 'use_sim_time': simulation,
        'image_source': 'ros2://' + rgb, 'depth_topic': depth, 'camera_info_topic': info,
        'depth_camera_info_topic': value('depth_camera_info_topic'),
    }
    if value('detectors'):
        parameters['detectors'] = string_list_argument(value('detectors'))
    for argument, name in (('yolo_device', 'device'), ('yolo_model', 'model_name')):
        if value(argument):
            parameters['yolo.' + name] = value(argument)
    return Node(package='mira2_vision', executable='mira2_vision', name='perception',
                namespace=value('namespace'), output='screen', parameters=[value('config'), parameters])


def optical_flow(value, simulation):
    parameters = {'simulation': simulation, 'use_sim_time': simulation}
    if simulation:
        parameters.update(image_source='ros2:///vision/bottom/image',
                          camera_info_topic='/vision/bottom/camera_info', camera_frame='bottom_camera_optical')
    if value('optical_flow_floor_depth'):
        parameters['optical_flow.floor_depth'] = float(value('optical_flow_floor_depth'))
    configs = [value('flow_config')]
    if value('flow_scene_config'):
        configs.append(value('flow_scene_config'))
    return Node(package='mira2_vision', executable='mira2_vision', name='optical_flow',
                namespace='mira/perception/bottom_flow', output='screen', parameters=[*configs, parameters])


def imu_parameters(value, simulation):
    """slam_imu:=true enables cuVSLAM visual-inertial fusion (experimental, off)."""
    if value('slam_imu') != 'true':
        return {}
    imu = value('imu_topic') or ('/vision/imu' if simulation else '')
    if not imu:
        raise ValueError('slam_imu needs imu_topic on hardware')
    return {'cuvslam.imu_topic': imu, 'cuvslam.tracking_mode': 'multisensor'}


def odometry(value, simulation, topics, backend):
    rgb, depth, info = topics
    if backend == 'cuvslam':
        return [Node(package='mira2_vision', executable='cuvslam_odometry',
                     name='cuvslam_odometry', namespace='mira/visual_slam', output='screen',
                     parameters=[value('rgbd_config'), value('vslam_config'), {
                         'use_sim_time': simulation,
                         **({'cuvslam.tracking_mode': 'multisensor'} if simulation else {}),
                         **imu_parameters(value, simulation),
                         'rgbd_inputs.rgb_topic': rgb, 'rgbd_inputs.depth_topic': depth,
                         'rgbd_inputs.camera_info_topic': info,
                     }])]
    if value('slam_imu') == 'true':
        raise ValueError('slam_imu requires slam_algorithm cuvslam')
    if backend == 'rtabmap':
        estimator = Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{'use_sim_time': simulation, 'frame_id': 'base_link', 'odom_frame_id': 'vo_odom',
                         'publish_tf': False, 'wait_for_transform': 0.2, 'approx_sync': True,
                         'approx_sync_max_interval': 0.01, 'publish_null_when_lost': True,
                         'Odom/ResetCountdown': '0'}],
            remappings=[('rgb/image', rgb), ('depth/image', depth),
                        ('rgb/camera_info', info), ('odom', '/vision/vo_odom')])
    return [estimator]


def launch_navigation(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    simulation = value('simulation') == 'true'
    source = pose_source(value)
    topics = camera_topics(value, simulation)
    nodes = [perception(value, simulation, topics)]
    if source == 'optical_flow' or (source == 'slam' and value('slam_optical_flow') == 'true'):
        nodes.append(optical_flow(value, simulation))
    if source == 'slam' and value('start_vo') == 'true':
        backend = select_backend(value('vo_backend') or value('slam_algorithm'))
        nodes = [LogInfo(msg=f'ROS odometry backend: {backend}'), *nodes,
                 *odometry(value, simulation, topics, backend)]
    return nodes


def generate_launch_description():
    config = lambda name: PathJoinSubstitution([FindPackageShare('mira2_vision'), 'config', name])
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in NAVIGATION_ARGUMENTS.items()],
        DeclareLaunchArgument('config', default_value=config('navigation.yaml')),
        DeclareLaunchArgument('flow_config', default_value=config('optical_flow.yaml')),
        DeclareLaunchArgument('vslam_config', default_value=config('cuvslam.yaml')),
        DeclareLaunchArgument('rgbd_config', default_value=config('rgbd_inputs.yaml')),
        OpaqueFunction(function=launch_navigation),
    ])
