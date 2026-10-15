"""In-process NVIDIA cuVSLAM RGB-D odometry, the GPU alternative to RTAB-Map.

Publishes nav_msgs/Odometry with RTAB-Map's conventions: the pose is base_link
in an odometry frame equal to base_link at the start of tracking, the twist is
in base_link, and lost tracking is signalled by LOST covariance. Consumers of
the odometry topic therefore need no changes when the backend is swapped.
"""
import ctypes
import glob
import os
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

LOST_COVARIANCE = 9999.0
# Without features cuVSLAM keeps returning the last pose with zero covariance
# instead of failing, so too few tracked observations also counts as lost.
MIN_OBSERVATIONS = 15
# cuVSLAM wheels link these CUDA 12 math libraries without bundling them; the
# PyTorch CUDA wheels install them as nvidia-* packages beside it.
CUDA_LIBRARIES = ('cublas', 'cusparse', 'nvjitlink', 'cusolver')


def load_cuvslam():
    """Import cuvslam, preloading pip-installed CUDA libraries when needed."""
    try:
        import cuvslam
    except ImportError as error:
        if 'cannot open shared object file' not in str(error):
            raise
        for name in CUDA_LIBRARIES:
            for root in sys.path:
                for path in sorted(glob.glob(os.path.join(root, 'nvidia', name, 'lib', 'lib*.so.*'))):
                    ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
        import cuvslam
    return cuvslam


def probe():
    """Return None when cuVSLAM can track on this machine, else the reason."""
    try:
        load_cuvslam().warm_up_gpu()
    except Exception as error:  # noqa: BLE001 - any import/CUDA failure means fall back
        return f'{type(error).__name__}: {error}'
    return None


def camera_model(cuvslam, matrix, size, dist_coeffs):
    """Build a cuVSLAM camera from an OpenCV intrinsic matrix and distortion."""
    camera = cuvslam.Camera()
    camera.size = size
    camera.focal = (float(matrix[0, 0]), float(matrix[1, 1]))
    camera.principal = (float(matrix[0, 2]), float(matrix[1, 2]))
    coefficients = np.zeros(8) if dist_coeffs is None else np.ravel(dist_coeffs)[:8]
    if np.any(coefficients):
        # cuVSLAM's polynomial model takes OpenCV's first eight coefficients.
        parameters = np.pad(coefficients, (0, 8 - coefficients.size)).tolist()
        camera.distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Polynomial, parameters)
    return camera


class CuVslamOdometry:
    def __init__(self, node, odom_topic, max_depth=8.0, odom_frame='vo_odom', base_frame='base_link'):
        self.cuvslam = load_cuvslam()
        self.node = node
        # Far depth is treated as exact by RGB-D tracking, but its error grows with
        # range: in Gazebo, 9-20 m depth creeps by centimetres while the scene is
        # static, which cuVSLAM reads as motion (40 cm in 20 s stationary; 0 with
        # this cap). Pixels beyond max_depth are passed as invalid.
        self.max_depth = max_depth
        self.odom_frame, self.base_frame = odom_frame, base_frame
        self.publisher = node.create_publisher(Odometry, odom_topic, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tracker = None
        self.previous = None
        self.last_stamp = None
        self.lost = 0

    def _build(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame.header.frame_id, Time())
        except Exception as error:  # noqa: BLE001 - tf2 raises several types
            self.node.get_logger().warn(f'cuVSLAM waiting for {self.base_frame} <- '
                                        f'{frame.header.frame_id}: {error}', throttle_duration_sec=5.0)
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        height, width = frame.image.shape[:2]
        camera = camera_model(self.cuvslam, frame.matrix, (width, height), self.node.dist_coeffs)
        camera.rig_from_camera = self.cuvslam.Pose([q.x, q.y, q.z, q.w], [t.x, t.y, t.z])
        settings = self.cuvslam.Tracker.OdometryRGBDSettings()
        settings.depth_scale_factor = 1000.0  # millimetre depth, as sent below
        settings.depth_camera_id = 0
        config = self.cuvslam.Tracker.OdometryConfig(
            odometry_mode=self.cuvslam.Tracker.OdometryMode.RGBD, rgbd_settings=settings,
            use_gpu=True, async_sba=True)
        # The rig frame is base_link, so poses come out as base_link in odom.
        self.tracker = self.cuvslam.Tracker(self.cuvslam.Rig([camera]), config)
        self.node.get_logger().info(f'cuVSLAM {self.cuvslam.get_version()[0]} RGB-D odometry '
                                    f'tracking {width}x{height} on the GPU')
        return True

    def process(self, frame):
        if frame.depth is None or frame.matrix is None:
            return
        if self.tracker is None and not self._build(frame):
            return
        stamp = frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return  # cuVSLAM rejects non-increasing timestamps (e.g. sim time reset)
        self.last_stamp = stamp
        rgb = np.ascontiguousarray(frame.image[:, :, ::-1])
        depth = np.nan_to_num(frame.depth.image * 1000.0, nan=0.0, posinf=0.0)
        depth[depth > self.max_depth * 1000.0] = 0.0
        depth = np.clip(depth, 0, 65535).astype(np.uint16)
        try:
            estimate, _ = self.tracker.track(stamp, images=[rgb], depths=[depth])
        except ValueError as error:
            self.node.get_logger().warn(f'cuVSLAM rejected frame: {error}', throttle_duration_sec=5.0)
            return
        message = Odometry()
        message.header.stamp = frame.header.stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        observations = len(self.tracker.get_last_observations(0))
        if estimate.world_from_rig is None or observations < MIN_OBSERVATIONS:
            self.lost += 1
            self.previous = None
            message.pose.pose.orientation.w = 1.0
            message.pose.covariance[0::7] = [LOST_COVARIANCE] * 6
            message.twist.covariance[0::7] = [LOST_COVARIANCE] * 6
            self.node.get_logger().warn(f'cuVSLAM lost tracking ({observations} features)',
                                        throttle_duration_sec=2.0)
            self.publisher.publish(message)
            return
        pose = estimate.world_from_rig.pose
        position = np.asarray(pose.translation, float)
        rotation = Rotation.from_quat(pose.rotation)
        p, o = message.pose.pose.position, message.pose.pose.orientation
        p.x, p.y, p.z = position.tolist()
        o.x, o.y, o.z, o.w = rotation.as_quat().tolist()
        message.pose.covariance = [float(v) for v in estimate.world_from_rig.covariance_xyz_rpy]
        if self.previous is not None:
            last_stamp, last_position, last_rotation = self.previous
            dt = (stamp - last_stamp) * 1e-9
            if dt > 0:
                # Body-frame (base_link) rates, matching RTAB-Map's twist frame.
                linear = rotation.inv().apply(position - last_position) / dt
                angular = (last_rotation.inv() * rotation).as_rotvec() / dt
                v, w = message.twist.twist.linear, message.twist.twist.angular
                v.x, v.y, v.z = linear.tolist()
                w.x, w.y, w.z = angular.tolist()
        self.previous = (stamp, position, rotation)
        self.publisher.publish(message)

    def close(self):
        self.node.destroy_publisher(self.publisher)
