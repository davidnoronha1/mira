"""In-process NVIDIA cuVSLAM RGB-D (optionally visual-inertial) odometry and native SLAM.

Publishes nav_msgs/Odometry with RTAB-Map's conventions: the pose is base_link
in an odometry frame equal to base_link at the start of tracking, the twist is
in base_link, and lost tracking is signalled by LOST covariance. Consumers of
the odometry topic therefore need no changes when the backend is swapped.
"""
from collections import deque
import ctypes
import glob
import os
import sys
import threading

import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

LOST_COVARIANCE = 9999.0
# Without features cuVSLAM keeps returning the last pose with zero covariance
# instead of failing, so too few tracked observations also counts as lost.
MIN_OBSERVATIONS = 15
# cuVSLAM wheels link these CUDA 12 math libraries without bundling them; the
# PyTorch CUDA wheels install them as nvidia-* packages beside it.
CUDA_LIBRARIES = ('cublas', 'cusparse', 'nvjitlink', 'cusolver')
# Default IMU noise (continuous-time densities / random walks) of a typical
# MEMS IMU such as the Pixhawk's ICM-20602/BMI088. Override with measured
# Allan-variance values for the real vehicle.
IMU_NOISE = dict(gyroscope_noise_density=1.7e-4, gyroscope_random_walk=1.9e-5,
                 accelerometer_noise_density=2.0e-3, accelerometer_random_walk=3.0e-3)
IMU_BUFFER = 4000  # >2 s at 1 kHz; covers the gap between depth-matched frames


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
    def __init__(self, node, odom_topic, max_depth=8.0, odom_frame='vo_odom', base_frame='base_link',
                 enable_slam=True, tracking_mode='rgbd', imu_topic='', imu_frame='', imu_noise=None):
        if tracking_mode not in ('rgbd', 'multisensor'):
            raise ValueError('tracking_mode must be rgbd or multisensor')
        if imu_topic and tracking_mode != 'multisensor':
            raise ValueError('cuVSLAM IMU fusion requires tracking_mode multisensor')
        unknown = set(imu_noise or {}) - IMU_NOISE.keys()
        if unknown:
            raise ValueError(f'Unknown cuVSLAM IMU noise settings: {sorted(unknown)}')
        self.tracking_mode = tracking_mode
        self.enable_slam = enable_slam
        self.cuvslam = load_cuvslam()
        self.node = node
        # Far depth is treated as exact by RGB-D tracking, but its error grows with
        # range: in Gazebo, 9-20 m depth creeps by centimetres while the scene is
        # static, which cuVSLAM reads as motion (40 cm in 20 s stationary; 0 with
        # this cap). Pixels beyond max_depth are passed as invalid.
        self.max_depth = max_depth
        self.odom_frame, self.base_frame = odom_frame, base_frame
        self.publisher = node.create_publisher(Odometry, odom_topic, 1)
        self.slam_publisher = (node.create_publisher(PoseStamped, odom_topic + '/slam_pose', 1)
                               if enable_slam else None)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        # Visual-inertial fusion: cuVSLAM's Multisensor mode fuses an IMU once the
        # rig contains one. Samples are buffered here and registered in time
        # order before the frame they precede.
        self.imu_frame, self.imu_noise = imu_frame, {**IMU_NOISE, **(imu_noise or {})}
        self.imu_samples = deque(maxlen=IMU_BUFFER)
        self.imu_lock = threading.Lock()
        self.imu_registered = None
        self.imu_subscription = (node.create_subscription(Imu, imu_topic, self._on_imu, QoSProfile(depth=200))
                                 if imu_topic else None)
        self.tracker = None
        self.previous = None
        self.last_stamp = None
        self.lost = 0

    def _on_imu(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        a, w = message.linear_acceleration, message.angular_velocity
        if not all(np.isfinite((a.x, a.y, a.z, w.x, w.y, w.z))):
            return
        with self.imu_lock:
            if self.imu_samples and stamp <= self.imu_samples[-1][0]:
                return  # Duplicate or reset clock; cuVSLAM needs increasing time.
            self.imu_samples.append((stamp, message.header.frame_id, (a.x, a.y, a.z), (w.x, w.y, w.z)))

    def _imu_calibration(self):
        """Return the rig IMU from TF and observed rate, or None while waiting."""
        with self.imu_lock:
            samples = list(self.imu_samples)
        if len(samples) < 50:
            self.node.get_logger().warn('cuVSLAM waiting for IMU samples', throttle_duration_sec=5.0)
            return None
        frame = self.imu_frame or samples[-1][1]
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except Exception as error:  # noqa: BLE001 - tf2 raises several types
            self.node.get_logger().warn(f'cuVSLAM waiting for {self.base_frame} <- {frame}: {error}',
                                        throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        frequency = (len(samples) - 1) / ((samples[-1][0] - samples[0][0]) * 1e-9)
        imu = self.cuvslam.ImuCalibration(
            rig_from_imu=self.cuvslam.Pose([q.x, q.y, q.z, q.w], [t.x, t.y, t.z]),
            frequency=float(frequency), **self.imu_noise)
        self.node.get_logger().info(f'cuVSLAM IMU fusion: {frame} at {frequency:.0f} Hz')
        return imu

    def _register_imu(self, previous, stamp):
        """Register buffered IMU samples between the last frame and this one, in order.

        cuVSLAM needs one non-decreasing stream of IMU and frame times, so a sample
        that arrives after its frame was already tracked is dropped.
        """
        floor = max(t for t in (previous, self.imu_registered, -1) if t is not None)
        with self.imu_lock:
            samples = [s for s in self.imu_samples if floor < s[0] <= stamp]
        for sample_stamp, _, acceleration, rate in samples:
            self.tracker.register_imu_measurement(0, self.cuvslam.ImuMeasurement(
                timestamp_ns=sample_stamp, linear_accelerations=acceleration, angular_velocities=rate))
            self.imu_registered = sample_stamp
        return len(samples)

    def _build(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame.header.frame_id, Time())
        except Exception as error:  # noqa: BLE001 - tf2 raises several types
            self.node.get_logger().warn(f'cuVSLAM waiting for {self.base_frame} <- '
                                        f'{frame.header.frame_id}: {error}', throttle_duration_sec=5.0)
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        height, width = frame.image.shape[:2]
        distortion = getattr(frame, 'dist_coeffs', getattr(self.node, 'dist_coeffs', None))
        camera = camera_model(self.cuvslam, frame.matrix, (width, height), distortion)
        camera.rig_from_camera = self.cuvslam.Pose([q.x, q.y, q.z, q.w], [t.x, t.y, t.z])
        self.base_from_camera = tf.transform
        self.camera_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        self.camera_translation = np.array([t.x, t.y, t.z])
        imus = []
        if self.imu_subscription is not None:
            imu = self._imu_calibration()
            if imu is None:
                return False
            imus = [imu]
        if self.tracking_mode == 'multisensor':
            # The unified depth solver constrains rotations on the repetitive
            # pool floor; legacy RGBD produced severe drift during spawn scans.
            if distortion is not None and np.any(distortion):
                raise ValueError('Multisensor requires rectified pinhole RGB-D inputs')
            settings = self.cuvslam.Tracker.OdometryMultisensorSettings()
            settings.depth_camera_ids = [0]
            settings.depth_scale_factor = 1000.0
            settings.enable_depth_stereo_tracking = True
            config = self.cuvslam.Tracker.OdometryConfig(
                odometry_mode=self.cuvslam.Tracker.OdometryMode.Multisensor,
                multisensor_settings=settings, use_gpu=True, async_sba=True,
                enable_observations_export=True)
        else:
            settings = self.cuvslam.Tracker.OdometryRGBDSettings()
            settings.depth_scale_factor = 1000.0
            settings.depth_camera_id = 0
            config = self.cuvslam.Tracker.OdometryConfig(
                odometry_mode=self.cuvslam.Tracker.OdometryMode.RGBD, rgbd_settings=settings,
                use_gpu=True, async_sba=True, enable_observations_export=True)
        slam_config = self.cuvslam.Tracker.SlamConfig() if self.enable_slam else None
        if slam_config is not None:
            slam_config.use_gpu = True
            slam_config.planar_constraints = False
        # Rig poses are base_link in the tracking-start frame. No navigation TF.
        self.tracker = self.cuvslam.Tracker(self.cuvslam.Rig([camera], imus), config, slam_config)
        self.node.get_logger().info(f'cuVSLAM {self.cuvslam.get_version()[0]} {self.tracking_mode} RGB-D'
                                    f'{"-inertial" if imus else ""} odometry'
                                    f'{" + native SLAM" if slam_config is not None else ""} '
                                    f'tracking {width}x{height} on the GPU')
        return True

    def process(self, frame):
        if frame.depth is None or frame.matrix is None:
            return
        if self.tracker is None and not self._build(frame):
            return
        stamp = frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return  # Drop duplicates/out-of-order frames without changing the odometry origin.
        previous, self.last_stamp = self.last_stamp, stamp
        rgb = np.ascontiguousarray(frame.image[:, :, ::-1])
        depth = np.nan_to_num(frame.depth.image * 1000.0, nan=0.0, posinf=0.0)
        depth[(depth > self.max_depth * 1000.0) | (depth <= 0)] = 0.0
        depth = np.clip(depth, 0, 65535).astype(np.uint16)
        try:
            if self.imu_subscription is not None and not self._register_imu(previous, stamp):
                self.node.get_logger().warn('cuVSLAM frame without new IMU samples', throttle_duration_sec=5.0)
            estimate, slam_pose = self.tracker.track(stamp, images=[rgb], depths=[depth])
        except ValueError as error:
            self.node.get_logger().warn(f'cuVSLAM rejected frame: {error}', throttle_duration_sec=5.0)
            self._publish_lost(frame.header)
            return
        message = Odometry()
        message.header.stamp = frame.header.stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        observations = len(self.tracker.get_last_observations(0))
        if estimate.world_from_rig is None or observations < MIN_OBSERVATIONS:
            self._publish_lost(frame.header)
            return
        pose = estimate.world_from_rig.pose
        position = np.asarray(pose.translation, float)
        covariance = np.asarray(estimate.world_from_rig.covariance_xyz_rpy, float).reshape(-1)
        if (not np.isfinite(position).all() or not np.isfinite(pose.rotation).all()
                or not np.isfinite(covariance).all() or np.linalg.norm(pose.rotation) < 1e-6):
            self._publish_lost(frame.header)
            return
        rotation = Rotation.from_quat(pose.rotation)
        p, o = message.pose.pose.position, message.pose.pose.orientation
        p.x, p.y, p.z = position.tolist()
        o.x, o.y, o.z, o.w = rotation.as_quat().tolist()
        message.pose.covariance = covariance.tolist()
        message.twist.covariance[0::7] = [LOST_COVARIANCE] * 6
        if self.previous is not None:
            last_stamp, last_position, last_rotation, last_covariance = self.previous
            dt = (stamp - last_stamp) * 1e-9
            if dt > 0:
                # Body-frame (base_link) rates, matching RTAB-Map's twist frame.
                linear = rotation.inv().apply(position - last_position) / dt
                angular = (last_rotation.inv() * rotation).as_rotvec() / dt
                # Independent-pose approximation; never claim perfect velocity certainty.
                message.twist.covariance = ((covariance + last_covariance) / dt**2).tolist()
                v, w = message.twist.twist.linear, message.twist.twist.angular
                v.x, v.y, v.z = linear.tolist()
                w.x, w.y, w.z = angular.tolist()
        self.previous = (stamp, position, rotation, covariance)
        self.publisher.publish(message)

        if self.slam_publisher is not None and slam_pose is not None:
            mapped = PoseStamped()
            mapped.header.stamp = frame.header.stamp
            mapped.header.frame_id = self.odom_frame + '_map'
            mapped.pose.position.x, mapped.pose.position.y, mapped.pose.position.z = map(float, slam_pose.translation)
            q = mapped.pose.orientation
            q.x, q.y, q.z, q.w = map(float, slam_pose.rotation)
            self.slam_publisher.publish(mapped)

    def _publish_lost(self, header):
        self.lost += 1
        self.previous = None
        message = Odometry()
        message.header.stamp = header.stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0::7] = [LOST_COVARIANCE] * 6
        message.twist.covariance[0::7] = [LOST_COVARIANCE] * 6
        self.publisher.publish(message)
        self.node.get_logger().warn('cuVSLAM lost tracking', throttle_duration_sec=2.0)

    def close(self):
        self.tracker = None
        self.tf_listener.unregister()
        if self.imu_subscription is not None:
            self.node.destroy_subscription(self.imu_subscription)
        self.node.destroy_publisher(self.publisher)
        if self.slam_publisher is not None:
            self.node.destroy_publisher(self.slam_publisher)
