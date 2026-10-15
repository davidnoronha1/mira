"""Shared RGB-D -> visual odometry; odometry -> external navigation.

RGB and depth are synchronized by the camera worker, not independently decoded
by this consumer. vo.backend selects the estimator: rtabmap republishes RGB-D
for the separate RTAB-Map process (CPU), cuvslam tracks in-process on the GPU.
Both publish the same nav_msgs/Odometry on vo.odom_topic.
"""
import collections
import copy
import math
import time
from types import SimpleNamespace
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from utils.navigation import ENU_TO_NED, FLU_TO_FRD, quat_to_rot, rot_to_quat_wxyz, rot_z, wrap_pi
from utils.parameters import declare_config, Setting, choices, positive
from mira2_vision.plugin import Plugin
from mira2_vision.consumers.mavlink import SensorLink, mavutil

SRC_TO_NED = {'gt': ENU_TO_NED, 'vo': FLU_TO_FRD}
YAW_SETTLE_S, YAW_SETTLE_DEG = 3.0, 0.5
LOST_COVARIANCE = 1000.0
NAN21 = [float('nan')] * 21


def runtime_options(config, simulation):
    """Remove simulation assumptions even when a sim YAML is reused on hardware."""
    args = SimpleNamespace(**config)
    if not simulation:
        args.source = 'vo'
        args.start_x = args.start_y = 0.0
        # A surveyed origin is opt-in, never the simulator's coordinates.
        args.set_origin = args.set_origin and args.origin_configured
    return args


class VisionBridge(Plugin):
    def __init__(self, node):
        super().__init__(node)
        if not self.settings['depth_topic']:
            raise ValueError('vision_bridge requires a depth_topic')
        simulation = self.settings['simulation']
        config = declare_config(node, {
            'source': Setting('vo', choices('vo', 'gt')),
            'backend': Setting('rtabmap', choices('rtabmap', 'cuvslam')),
            'cuvslam_max_depth': Setting(8.0, positive),
            'mavlink': 'udpin:0.0.0.0:14555' if simulation else '',
            'odom_topic': '/vision/vo_odom', 'gt_topic': '/vision/gt_odom',
            'start_x': 0.0, 'start_y': 0.0, 'no_align': False,
            'set_origin': simulation, 'origin_configured': False,
            'origin_lat': 63.52 if simulation else 0.0,
            'origin_lon': 10.35 if simulation else 0.0, 'origin_alt': 0.0,
        }, namespace='vo')
        self.args = runtime_options(config, simulation)
        if not all(math.isfinite(v) for v in (self.args.start_x, self.args.start_y,
                self.args.origin_lat, self.args.origin_lon, self.args.origin_alt)):
            raise ValueError('VO offsets and origin must be finite')
        if not -90 <= self.args.origin_lat <= 90 or not -180 <= self.args.origin_lon <= 180:
            raise ValueError('Invalid geographic origin')
        self.src_to_ned = SRC_TO_NED[self.args.source]
        self.offset_ned = np.array([self.args.start_y, self.args.start_x, 0.0]) if self.args.source == 'vo' else np.zeros(3)
        self.align = self.ahrs_yaw = None
        self.yaw_history = collections.deque()
        self.origin_set = False
        self.sent = self.dropped = 0
        self.last_depth_stamp = None
        self.estimator = None
        if self.args.source == 'vo' and self.args.backend == 'cuvslam':
            from mira2_vision.cuvslam_odometry import CuVslamOdometry
            self.estimator = CuVslamOdometry(node, self.args.odom_topic, self.args.cuvslam_max_depth)
        else:
            self.rgb_pub = node.create_publisher(Image, 'vo/rgb', 1)
            self.depth_pub = node.create_publisher(Image, 'vo/depth', 1)
            self.info_pub = node.create_publisher(CameraInfo, 'vo/camera_info', 1)
        self.link = SensorLink(node, self.args.mavlink,
                              mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, self._on_mav)
        self.mav = self.link.connection
        topic = self.args.gt_topic if self.args.source == 'gt' else self.args.odom_topic
        self.odom_sub = node.create_subscription(Odometry, topic, self._on_odom, 1)
        self.timer = node.create_timer(1.0, self._housekeeping)

    def process(self, frame):
        depth = frame.depth
        if depth is None or frame.matrix is None or self.args.source != 'vo':
            return
        stamp = (depth.header.stamp.sec, depth.header.stamp.nanosec)
        if stamp == self.last_depth_stamp:
            return
        self.last_depth_stamp = stamp
        if self.estimator is not None:
            self.estimator.process(frame)
            return
        rgb = self.node.bridge.cv2_to_imgmsg(frame.image, encoding='bgr8')
        rgb.header = frame.header
        info = copy.deepcopy(self.node.camera_info) if self.node.camera_info is not None else CameraInfo()
        info.header = frame.header
        info.height, info.width = frame.image.shape[:2]
        info.k = frame.matrix.reshape(-1).tolist()
        info.d = self.node.dist_coeffs.tolist() if self.node.dist_coeffs is not None else []
        info.distortion_model = info.distortion_model or 'plumb_bob'
        info.r = np.eye(3).reshape(-1).tolist()
        info.p = np.column_stack((frame.matrix, np.zeros(3))).reshape(-1).tolist()
        # Keep original acquisition stamps; RTAB-Map applies approximate sync.
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth.message)
        self.info_pub.publish(info)

    def _on_mav(self, msg):
        if msg.get_type() == 'ATTITUDE':
            self._on_attitude(msg.yaw)
        elif msg.get_type() == 'GPS_GLOBAL_ORIGIN':
            self.origin_set = time.monotonic()

    def close(self):
        if self.estimator is not None:
            self.estimator.close()
        self.node.destroy_timer(self.timer)
        self.node.destroy_subscription(self.odom_sub)
        self.link.close()

    def _on_attitude(self, yaw):
        # The EKF heading keeps converging for a while after boot, so only
        # expose it for alignment once it has held still for YAW_SETTLE_S.
        now = time.monotonic()
        self.yaw_history.append((now, yaw))
        while self.yaw_history[0][0] < now - YAW_SETTLE_S:
            self.yaw_history.popleft()
        span = self.yaw_history[-1][0] - self.yaw_history[0][0]
        ref = self.yaw_history[0][1]
        spread = max(abs(wrap_pi(y - ref)) for _, y in self.yaw_history)
        if span >= YAW_SETTLE_S * 0.9 and spread < math.radians(YAW_SETTLE_DEG):
            self.ahrs_yaw = yaw

    def _housekeeping(self):
        if not self.link.ready:
            return
        # Poll the origin so a rebooted autopilot (origin lost) gets it again
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
        if self.origin_set and time.monotonic() - self.origin_set > 5.0:
            self.node.get_logger().warn('EKF origin no longer reported (autopilot rebooted?), re-sending')
            self.origin_set = False
        if self.args.set_origin and not self.origin_set:
            self.mav.mav.set_gps_global_origin_send(
                self.mav.target_system,
                int(self.args.origin_lat * 1e7), int(self.args.origin_lon * 1e7),
                int(self.args.origin_alt * 1000), int(time.time() * 1e6))
        if self.sent or self.dropped:
            self.node.get_logger().info(
                f'sent {self.sent} / dropped {self.dropped} (lost tracking) in last 1s',
                throttle_duration_sec=10.0)
        self.sent = self.dropped = 0

    def _on_odom(self, msg):
        age = (self.node.get_clock().now().nanoseconds * 1e-9 -
               msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                  msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
        if (not all(math.isfinite(v) for v in values) or
                not math.isfinite(msg.pose.covariance[0]) or
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01 or
                age < -0.05 or age > 0.5 or
                msg.pose.covariance[0] >= LOST_COVARIANCE):
            self.dropped += 1
            return

        if not self.link.ready:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        r_src_body = quat_to_rot(q.x, q.y, q.z, q.w)
        r_ned_frd = self.src_to_ned @ r_src_body @ FLU_TO_FRD

        if self.align is None:
            if self.args.no_align:
                self.align = np.eye(3)
            elif self.ahrs_yaw is None:
                self.node.get_logger().warn('Waiting for AHRS heading to settle before aligning',
                                       throttle_duration_sec=5.0)
                return
            else:
                src_yaw = math.atan2(r_ned_frd[1, 0], r_ned_frd[0, 0])
                delta = wrap_pi(self.ahrs_yaw - src_yaw)
                self.align = rot_z(delta)
                self.node.get_logger().info(
                    f'Aligned {self.args.source} heading to AHRS: rotate {math.degrees(delta):+.1f} deg')

        pos = self.align @ self.src_to_ned @ np.array([p.x, p.y, p.z]) + self.offset_ned
        r_ned_frd = self.align @ r_ned_frd

        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        vel = FLU_TO_FRD @ np.array([lin.x, lin.y, lin.z])
        rates = FLU_TO_FRD @ np.array([ang.x, ang.y, ang.z])

        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self.mav.mav.odometry_send(
            stamp_us,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(pos[0]), float(pos[1]), float(pos[2]),
            rot_to_quat_wxyz(r_ned_frd),
            float(vel[0]), float(vel[1]), float(vel[2]),
            float(rates[0]), float(rates[1]), float(rates[2]),
            NAN21, NAN21,
            0,  # reset_counter
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
            100)  # quality
        self.sent += 1

