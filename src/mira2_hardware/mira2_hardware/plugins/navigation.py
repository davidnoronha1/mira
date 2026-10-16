"""Autopilot navigation feedback in ROS ENU/FLU, with optional path display."""
import math
import time

import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from pymavlink import mavutil
from tf2_ros import TransformBroadcaster
from utils.navigation import ENU_TO_NED, FLU_TO_FRD, rot_to_quat_wxyz
from utils import topics

from mira2_hardware.controller_plugin import ControllerPlugin
from mira2_hardware.path_history import PathHistory
from mira2_hardware.mavlink_io import populate_scaled_imu

M = mavutil.mavlink
PATH_STEP = 0.05


def euler_to_rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


class NavigationFeedback(ControllerPlugin):
    message_rates = {
        M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 30.0,
        M.MAVLINK_MSG_ID_ATTITUDE: 30.0,
        M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 30.0,
        M.MAVLINK_MSG_ID_SCALED_PRESSURE: 30.0,
        M.MAVLINK_MSG_ID_SCALED_IMU: 100.0,
    }

    def __init__(self, controller):
        super().__init__(controller)
        self.latest_pose = None
        self.last_feedback = 0.0
        self.attitude = None
        self.velocity = (0.0, 0.0, 0.0)
        self.ground_pressure = None
        self.visualize = self.options.visualize
        self.ekf_path = PathHistory(self.options.path_max_points, PATH_STEP)
        self.gt_path = PathHistory(self.options.path_max_points, PATH_STEP)
        self.last_paths = self.last_pressure_request = 0.0
        self.pub_pose = self.create_publisher(PoseStamped, topics.POSE, 10)
        # Attitude and depth even while the EKF has no horizontal position (x, y
        # are NaN): optical flow needs them to produce the velocity that gives
        # the EKF its first horizontal estimate.
        self.pub_attitude = self.create_publisher(PoseStamped, topics.ATTITUDE, 10)
        # Raw depth-sensor height (point.z = -depth, m), zeroed with ArduSub's own
        # surface calibration (BARO1_GND_PRESS). Independent of the EKF.
        self.pub_pressure = self.create_publisher(PointStamped, topics.DEPTH, 10)
        # Raw autopilot IMU (SCALED_IMU, base_link FLU), without the EKF's bias
        # estimate: optical flow removes rotation with it. Derotating with EKF
        # attitude changes instead closed a loop (a wrong EKF gyro bias became
        # fake flow velocity, which the EKF fused) that diverged at boot.
        self.pub_imu = self.create_publisher(Imu, topics.STATE_IMU, 50)
        self.ground_pressure = None
        self.pub_ekf_path = self.create_publisher(Path, topics.PATH, 1)
        self.pub_gt_path = self.create_publisher(Path, topics.GROUND_TRUTH_PATH, 1) if self.options.simulation else None
        self.tf = TransformBroadcaster(controller)
        if self.visualize and self.options.simulation:
            self.create_subscription(Odometry, topics.GROUND_TRUTH_ODOM, self._on_gt, 1)

    def on_connect(self):
        self.latest_pose = None
        self.last_feedback = 0.0
        self.attitude = None
        self.ground_pressure = None
        self._request_pressure()

    def _request_pressure(self):
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component, b'BARO1_GND_PRESS', -1)

    def tick(self, now):
        if self.ground_pressure is None and now - self.last_pressure_request >= 2.0:
            self._request_pressure()
            self.last_pressure_request = now
        if self.visualize and now - self.last_paths >= 1.0 / self.options.path_rate:
            self._publish_paths()
            self.last_paths = now

    def _publish_paths(self):
        for history, pub in ((self.ekf_path, self.pub_ekf_path),
                             (self.gt_path, self.pub_gt_path)):
            if pub is None or not pub.get_subscription_count():
                continue
            poses = history.snapshot()
            if not poses:
                continue
            path = Path()
            path.header.frame_id = 'odom'
            path.header.stamp = poses[-1].header.stamp
            path.poses = poses
            pub.publish(path)

    # ---- ArduSub -> ROS -------------------------------------------------
    def on_message(self, msg):
        t = msg.get_type()
        if t == 'ATTITUDE':
            self.attitude = (msg.roll, msg.pitch, msg.yaw)
        elif t == 'STATUSTEXT':
            self.get_logger().info(f'ArduSub: {msg.text}')
        elif t == 'PARAM_VALUE':
            if msg.param_id == 'BARO1_GND_PRESS' and msg.param_value > 0:
                self.ground_pressure = msg.param_value
        elif t == 'SCALED_PRESSURE':
            if self.ground_pressure is not None:
                point = PointStamped()
                point.header.frame_id = 'odom'
                point.header.stamp = self.get_clock().now().to_msg()
                # Fresh water, as ArduSub's default BARO_SPEC_GRAV.
                point.point.z = -(msg.press_abs * 100.0 - self.ground_pressure) / (1000.0 * 9.80665)
                self.pub_pressure.publish(point)
        elif t == 'SCALED_IMU':
            self._publish_imu(msg)
        elif t == 'GLOBAL_POSITION_INT':
            if self.attitude is not None:
                self._publish_attitude(msg.relative_alt * 1e-3)
        elif t == 'LOCAL_POSITION_NED' and self.attitude is not None:
            self.velocity = (msg.vx, msg.vy, msg.vz)
            self._publish_ekf(msg)

    def _publish_imu(self, msg):
        imu = Imu()
        imu.header.frame_id = 'base_link'
        imu.header.stamp = self.get_clock().now().to_msg()
        populate_scaled_imu(imu, msg, flu=True)
        imu.orientation_covariance[0] = -1.0
        self.pub_imu.publish(imu)

    def _publish_attitude(self, altitude):
        r_enu_flu = ENU_TO_NED.T @ euler_to_rot(*self.attitude) @ FLU_TO_FRD
        w, x, y, z = rot_to_quat_wxyz(r_enu_flu)
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = ps.pose.position.y = math.nan  # horizontal position unknown here
        ps.pose.position.z = altitude  # EKF height above origin (negative below the surface)
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = w, x, y, z
        self.pub_attitude.publish(ps)

    def _publish_ekf(self, pos):
        r_enu_flu = ENU_TO_NED.T @ euler_to_rot(*self.attitude) @ FLU_TO_FRD
        w, x, y, z = rot_to_quat_wxyz(r_enu_flu)
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos.y, pos.x, -pos.z
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = w, x, y, z
        self.latest_pose = ps
        self.last_feedback = time.monotonic()
        self.pub_pose.publish(ps)
        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z
        tf.transform.rotation = ps.pose.orientation
        self.tf.sendTransform(tf)
        if self.visualize:
            self.ekf_path.append(ps)

    def _on_gt(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        self.gt_path.append(ps)

