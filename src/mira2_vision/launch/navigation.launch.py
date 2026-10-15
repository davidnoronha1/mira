"""Camera-owned RGB-D consumers and a cuVSLAM (GPU) or RTAB-Map (CPU) estimator."""
from pathlib import Path
import subprocess
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from utils.parameters import string_list_argument
from launch.substitutions import PathJoinSubstitution


PROBE = ('from mira2_vision.cuvslam_odometry import probe; reason = probe(); '
         'print(reason or "", end=""); raise SystemExit(1 if reason else 0)')


def select_backend(requested):
    """auto picks cuVSLAM when it can track here, else RTAB-Map on the CPU."""
    if requested in ('rtabmap', 'cuvslam'):
        return requested, f'{requested} requested explicitly'
    if requested != 'auto':
        raise ValueError('vo_backend must be auto, cuvslam or rtabmap')
    # Probe in a child so the launch process never initializes CUDA.
    try:
        result = subprocess.run([sys.executable, '-c', PROBE], capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return 'rtabmap', 'cuVSLAM probe timed out'
    if result.returncode == 0:
        return 'cuvslam', 'cuVSLAM GPU tracker available'
    reason = (result.stdout or result.stderr).strip().splitlines()
    return 'rtabmap', f'cuVSLAM unavailable ({reason[-1] if reason else "probe failed"})'


def launch_navigation(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    simulation = value('simulation') == 'true'
    source = value('source') if simulation else 'vo'
    rgb = value('rgb_topic') or ('/vision/image' if simulation else '/mira/sensors/camera/front/image_raw')
    depth = value('depth_topic') or ('/vision/depth' if simulation else '/mira/sensors/camera/front/depth/image_raw')
    info = value('camera_info_topic') or ('/vision/camera_info' if simulation else '/mira/sensors/camera/front/camera_info')
    namespace = value('namespace').strip('/')
    topic = lambda suffix: '/' + '/'.join(filter(None, (namespace, suffix)))
    parameters = {
        'simulation': simulation, 'use_sim_time': simulation,
        'image_source': 'ros2://' + rgb, 'depth_topic': depth, 'camera_info_topic': info,
        'depth_camera_info_topic': value('depth_camera_info_topic'),
        'vo.source': source, 'vo.start_x': float(value('start_x')) if simulation else 0.0,
        'vo.start_y': float(value('start_y')) if simulation else 0.0,
    }
    if source != 'vo':
        backend, why = 'rtabmap', 'source is not vo'
    elif value('start_vo') != 'true':
        backend, why = 'rtabmap', 'start_vo=false, odometry comes from an external process'
    else:
        backend, why = select_backend(value('vo_backend'))
    parameters['vo.backend'] = backend
    if value('detectors'):
        # e.g. detectors:=gazebo_boxes in simulation; blank keeps the config file's list.
        parameters['detectors'] = string_list_argument(value('detectors'))
    for arg, parameter in (('yolo_device', 'device'), ('yolo_model', 'model_name')):
        if value(arg):
            parameters[parameter] = value(arg)
    # Blank links use plugin defaults: SITL ports in sim, no transport on hardware.
    for arg, parameter in (('vision_link', 'vo.mavlink'), ('obstacle_link', 'obstacles.mavlink')):
        if value(arg):
            parameters[parameter] = value(arg)
    gpu = backend == 'cuvslam'
    banner = '#' * 72
    nodes = [LogInfo(msg=f'\n{banner}\n##  VISUAL ODOMETRY: {"cuVSLAM on GPU" if gpu else "RTAB-Map on CPU"} - {why}\n{banner}')]
    nodes += [Node(package='mira2_vision', executable='mira2_vision', namespace=namespace,
                  name='perception', output='screen', parameters=[value('config'), parameters])]
    if value('start_vo') == 'true' and source == 'vo' and backend == 'rtabmap':
        nodes.append(Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{'use_sim_time': simulation, 'frame_id': 'base_link',
                'odom_frame_id': 'vo_odom', 'publish_tf': False, 'wait_for_transform': 0.2,
                'approx_sync': True, 'approx_sync_max_interval': 0.01,
                'publish_null_when_lost': True, 'Odom/ResetCountdown': '0'}],
            remappings=[('rgb/image', topic('vo/rgb')), ('depth/image', topic('vo/depth')),
                        ('rgb/camera_info', topic('vo/camera_info')), ('odom', '/vision/vo_odom')]))
    return nodes


def generate_launch_description():
    defaults = {
        'simulation': 'false', 'source': 'vo', 'start_vo': 'true', 'vo_backend': 'auto',
        'yolo_device': '', 'yolo_model': '', 'detectors': '',
        'namespace': 'mira/perception/front', 'start_x': '0', 'start_y': '0',
        'vision_link': '', 'obstacle_link': '', 'rgb_topic': '', 'depth_topic': '',
        'camera_info_topic': '', 'depth_camera_info_topic': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [FindPackageShare('mira2_vision'), 'config', 'navigation.yaml'])),
        OpaqueFunction(function=launch_navigation),
    ])
