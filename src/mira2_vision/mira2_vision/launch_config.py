"""Camera/SLAM launch arguments and vision backend selection."""
from ament_index_python.packages import get_package_prefix, PackageNotFoundError


NAVIGATION_ARGUMENTS = {
    'simulation': 'false', 'source': 'vo', 'start_vo': 'true', 'vo_backend': '',
    'pose_source': '', 'slam_algorithm': 'auto',
    'slam_optical_flow': 'false', 'slam_imu': 'false', 'imu_topic': '',
    'optical_flow_floor_depth': '', 'flow_scene_config': '',
    'yolo_device': '', 'yolo_model': '', 'detectors': '',
    'namespace': 'mira/perception/front', 'rgb_topic': '', 'depth_topic': '',
    'camera_info_topic': '', 'depth_camera_info_topic': '',
}


def pose_source(value):
    source = value('pose_source') or {'vo': 'slam', 'gt': 'gt', 'none': 'optical_flow'}.get(value('source'), value('source'))
    if source not in ('slam', 'optical_flow', 'gt'):
        raise ValueError('pose_source must be slam, optical_flow or gt')
    if source == 'gt' and value('simulation') != 'true':
        raise ValueError('Ground-truth navigation requires simulation:=true')
    return source


def select_backend(requested):
    candidates = ('cuvslam', 'rtabmap') if requested == 'auto' else (requested,)
    failures = []
    for name in candidates:
        if name == 'cuvslam':
            from mira2_vision.cuvslam_odometry import probe
            reason = probe()
            if reason is None:
                return name
            failures.append('cuVSLAM: ' + reason)
        elif name == 'rtabmap':
            try:
                get_package_prefix('rtabmap_odom')
                return name
            except PackageNotFoundError:
                failures.append('RTAB-Map: rtabmap_odom package missing')
        else:
            raise ValueError('slam_algorithm must be cuvslam, rtabmap or auto')
    raise RuntimeError('No usable SLAM backend: ' + '; '.join(failures) +
                       '. Install a matching NVIDIA cuVSLAM wheel/CUDA runtime, or '
                       'ros-jazzy-rtabmap-odom for CPU tracking.')
