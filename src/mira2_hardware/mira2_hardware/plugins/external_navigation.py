"""Forward ROS odometry, flow velocity and heading over the controller's MAVLink link."""
import collections
import math
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from utils.navigation import ENU_TO_NED, FLU_TO_FRD, quat_to_rot, rot_to_quat_wxyz, rot_z, wrap_pi
from utils.parameters import declare_config, Setting, choices
from utils import time as utime
from mira2_hardware.controller_plugin import ControllerPlugin

SRC_TO_NED = {'gt': ENU_TO_NED, 'vo': FLU_TO_FRD}
YAW_SETTLE_S, YAW_SETTLE_DEG = 3.0, 0.5
LOST_COVARIANCE = 1000.0
NAN21 = [float('nan')] * 21


def runtime_options(config, simulation):
    args = SimpleNamespace(**config)
    if not simulation:
        if args.source == 'gt':
            raise ValueError('external_navigation.source=gt requires simulation=true')
        args.start_x = args.start_y = 0.0
        args.set_origin = args.set_origin and args.origin_configured
    return args


class ExternalNavigation(ControllerPlugin):
    def __init__(self, controller):
        super().__init__(controller)
        node = controller
        simulation = self.options.simulation
        config = declare_config(node, {
            # Pose fed to the autopilot EKF: visual SLAM, sim ground truth, or
            # none (optical-flow velocity only; see flow_topic/ekf_source_set).
            'source': Setting('vo', choices('vo', 'gt', 'none')),
            # Body velocity (TwistWithCovarianceStamped, e.g. optical_flow plugin)
            # forwarded as VISION_POSITION_DELTA (EKF body odometry). '' disables.
            'flow_topic': '',
            # Heading (PoseWithCovarianceStamped, odom ENU; orientation and yaw
            # variance only, e.g. optical_flow's floor-line heading) forwarded as
            # an ExternalNav yaw (VISION_POSITION_ESTIMATE) when source is none.
            # Its position is not fused: the EKF source set has no ExtNav XY.
            'heading_topic': '',
            # EKF3 source set selected at runtime (MAV_CMD_SET_EKF_SOURCE_SET, not
            # saved to storage); 0 leaves the autopilot's choice alone.
            'ekf_source_set': Setting(0, choices(0, 1, 2, 3)),
            'odom_topic': '/vision/vo_odom', 'gt_topic': '/vision/gt_odom',
            'start_x': 0.0, 'start_y': 0.0, 'no_align': False,
            'set_origin': simulation, 'origin_configured': False,
            'origin_lat': 63.52 if simulation else 0.0,
            'origin_lon': 10.35 if simulation else 0.0, 'origin_alt': 0.0,
        }, namespace='external_navigation')
        self.args = runtime_options(config, simulation)
        if not all(math.isfinite(v) for v in (self.args.start_x, self.args.start_y,
                self.args.origin_lat, self.args.origin_lon, self.args.origin_alt)):
            raise ValueError('VO offsets and origin must be finite')
        if not -90 <= self.args.origin_lat <= 90 or not -180 <= self.args.origin_lon <= 180:
            raise ValueError('Invalid geographic origin')
        self.src_to_ned = SRC_TO_NED.get(self.args.source)
        self.offset_ned = np.array([self.args.start_y, self.args.start_x, 0.0]) if self.args.source == 'vo' else np.zeros(3)
        self.last_flow_stamp = None
        self.flow_sent = 0
        self.attitude = self.vz_ned = None  # autopilot state for flow forwarding
        self.pressure = collections.deque()  # (time s, depth m) of the depth sensor, last 1 s
        self.align = self.ahrs_yaw = None
        self.yaw_history = collections.deque()
        self.origin_set = False
        self.sent = self.dropped = 0
        topic = self.args.gt_topic if self.args.source == 'gt' else self.args.odom_topic
        self.odom_sub = (node.create_subscription(Odometry, topic, self._on_odom, 1)
                         if self.args.source != 'none' else None)
        self.flow_sub = (node.create_subscription(TwistWithCovarianceStamped, self.args.flow_topic, self._on_flow, 10)
                         if self.args.flow_topic else None)
        self.heading_sub = (node.create_subscription(PoseWithCovarianceStamped, self.args.heading_topic,
                                                     self._on_heading, 10)
                            if self.args.heading_topic and self.args.source == 'none' else None)
        self.heading_sent = 0
        self.local_ned = None  # EKF position, echoed in heading messages
        self.last_housekeeping = 0.0

    message_rates = {
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 30.0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 30.0,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE: 30.0,
    }

    def on_connect(self):
        self.align = self.ahrs_yaw = None
        self.yaw_history.clear()
        self.origin_set = False
        self.last_flow_stamp = None
        self.pressure.clear()
        self.vz_ned = self.attitude = self.local_ned = None

    def tick(self, now):
        if now - self.last_housekeeping >= 1.0:
            self._housekeeping()
            self.last_housekeeping = now

    def on_message(self, msg):
        kind = msg.get_type()
        if kind == 'ATTITUDE':
            self.attitude = (msg.roll, msg.pitch, msg.yaw)
            self._on_attitude(msg.yaw)
        elif kind == 'SCALED_PRESSURE':
            self._on_pressure(msg.time_boot_ms, msg.press_abs)
        elif kind == 'LOCAL_POSITION_NED':
            self.local_ned = (msg.x, msg.y, msg.z)
        elif kind == 'GPS_GLOBAL_ORIGIN':
            self.origin_set = time.monotonic()

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
        if not self.ready:
            return
        # Poll the origin so a rebooted autopilot (origin lost) gets it again
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
        if self.origin_set and time.monotonic() - self.origin_set > 5.0:
            self.controller.get_logger().warn('EKF origin no longer reported (autopilot rebooted?), re-sending')
            self.origin_set = False
        if self.args.ekf_source_set:
            # Idempotent; repeated so a rebooted autopilot returns to this set.
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 0, self.args.ekf_source_set, 0, 0, 0, 0, 0, 0)
        if self.args.set_origin and not self.origin_set:
            self.master.mav.set_gps_global_origin_send(
                self.master.target_system,
                int(self.args.origin_lat * 1e7), int(self.args.origin_lon * 1e7),
                int(self.args.origin_alt * 1000), int(time.time() * 1e6))
        if self.heading_sent:
            self.controller.get_logger().info(f'{self.heading_sent} floor headings in last 1s', throttle_duration_sec=10.0)
            self.heading_sent = 0
        if self.sent or self.dropped or self.flow_sent:
            self.controller.get_logger().info(
                f'sent {self.sent} / dropped {self.dropped} (lost tracking) poses, '
                f'{self.flow_sent} flow velocities in last 1s', throttle_duration_sec=10.0)
        self.sent = self.dropped = self.flow_sent = 0

    def _on_pressure(self, time_ms, pressure):
        """Vertical velocity (down positive) from the depth sensor: 1 s line fit.

        An independent measurement for the vertical part of the flow message. The
        EKF's own vertical velocity must not be used: fed back with a delay as the
        only aiding source, it made the depth estimate run away (59 m error). A
        two-sample difference is no good either: SITL pressure depth has ~0.07 m
        sample noise, and the resulting vertical velocity noise corrupted the EKF
        depth (0.27 m scatter). Only the slope matters, so no surface reference.
        """
        t = time_ms * 1e-3
        if self.pressure and t < self.pressure[-1][0]:
            self.pressure.clear()  # autopilot rebooted
        self.pressure.append((t, pressure * 100.0 / (1000.0 * 9.80665)))
        while self.pressure[0][0] < t - 1.0:
            self.pressure.popleft()
        samples = np.array(self.pressure)
        if len(samples) >= 5 and samples[-1, 0] - samples[0, 0] >= 0.5:
            self.vz_ned = float(np.polyfit(samples[:, 0] - t, samples[:, 1], 1)[0])

    def _on_flow(self, msg):
        """Forward optical-flow body velocity to the EKF.

        With a SLAM pose (absolute aiding) it is a horizontal ExternalNav velocity
        (VISION_SPEED_ESTIMATE; EKF3 ignores its vertical part while the vertical
        velocity source is none). Without one, only body odometry
        (VISION_POSITION_DELTA) gives EKF3 relative aiding, at the cost of a
        vertical component that must come from the depth sensor.
        """
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        last, self.last_flow_stamp = self.last_flow_stamp, stamp
        if last is None or not 0 < stamp - last < 0.5 or not self.ready or self.attitude is None:
            return
        v = msg.twist.twist.linear
        sigma = math.sqrt(max(msg.twist.covariance[0], msg.twist.covariance[7]))
        if not all(math.isfinite(c) for c in (v.x, v.y, sigma)):
            return
        roll, pitch, yaw = self.attitude
        r_ned_frd = Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
        if self.args.source != 'none':
            velocity = r_ned_frd @ FLU_TO_FRD @ np.array([v.x, v.y, 0.0])
            variance = sigma ** 2
            self.master.mav.vision_speed_estimate_send(
                int(stamp * 1e6), float(velocity[0]), float(velocity[1]), 0.0,
                [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance], 0)
            self.flow_sent += 1
            return
        if self.vz_ned is None:
            return
        dt = stamp - last
        # EKF3 fuses body odometry as a full 3-D body velocity with one error
        # for all axes. Flow observes only horizontal motion, so the vertical
        # component comes from the depth sensor's pressure rate. With 0 here the
        # EKF lost depth and the vehicle surfaced in simulation; with the EKF's
        # own vertical velocity the depth estimate ran away in flow-only mode.
        horizontal = FLU_TO_FRD @ np.array([v.x, v.y, 0.0])
        vz = (self.vz_ned - r_ned_frd[2, 0] * horizontal[0] - r_ned_frd[2, 1] * horizontal[1]) / r_ned_frd[2, 2]
        delta = np.array([horizontal[0], horizontal[1], vz]) * dt
        # EKF3 maps confidence linearly to EK3_VIS_VERR_MIN..MAX (0.1..0.5 m/s).
        confidence = 100.0 * (0.5 - min(max(sigma, 0.1), 0.5)) / 0.4
        self.master.mav.vision_position_delta_send(
            int(stamp * 1e6), int(dt * 1e6), [0.0, 0.0, 0.0],
            [float(delta[0]), float(delta[1]), float(delta[2])], float(confidence))
        self.flow_sent += 1

    def _on_heading(self, msg):
        """Forward a heading as ExternalNav yaw (EK3_SRCn_YAW 6 in the flow source set).

        EKF3 fuses the yaw of VISION_POSITION_ESTIMATE whenever the yaw source is
        ExternalNav, independently of its position, but drops the whole message
        if the position is NaN. The position sent is the EKF's own latest
        estimate, so that it is a no-op if fused: source set 1 (active at boot,
        until ekf_source_set takes effect, and again after an autopilot reboot)
        fuses ExtNav XY, and a dummy (0, 0) there yanked the unaided position
        tens of metres (vehicle chased it 24 m across the pool).
        """
        q = msg.pose.pose.orientation
        variance = msg.pose.covariance[35]
        if (not self.ready or self.local_ned is None or
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, variance, *self.local_ned))):
            return
        r_ned_frd = ENU_TO_NED @ quat_to_rot(q.x, q.y, q.z, q.w) @ FLU_TO_FRD
        yaw, pitch, roll = Rotation.from_matrix(r_ned_frd).as_euler('ZYX')
        covariance = [0.0] * 21
        covariance[0] = covariance[6] = covariance[11] = 1e4  # position: the EKF's own, weakly
        covariance[20] = variance  # ArduPilot: angle error = sqrt(cov[15] + cov[18] + cov[20])
        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        x, y, z = self.local_ned
        self.master.mav.vision_position_estimate_send(stamp_us, float(x), float(y), float(z),
                                                   float(roll), float(pitch), float(yaw), covariance, 0)
        self.heading_sent += 1

    def _on_odom(self, msg):
        age = utime.stamp_age(self.get_clock(), msg.header.stamp)
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

        if not self.ready:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        r_src_body = quat_to_rot(q.x, q.y, q.z, q.w)
        r_ned_frd = self.src_to_ned @ r_src_body @ FLU_TO_FRD

        if self.align is None:
            if self.args.no_align:
                self.align = np.eye(3)
            elif self.ahrs_yaw is None:
                self.controller.get_logger().warn('Waiting for AHRS heading to settle before aligning',
                                       throttle_duration_sec=5.0)
                return
            else:
                src_yaw = math.atan2(r_ned_frd[1, 0], r_ned_frd[0, 0])
                delta = wrap_pi(self.ahrs_yaw - src_yaw)
                self.align = rot_z(delta)
                self.controller.get_logger().info(
                    f'Aligned {self.args.source} heading to AHRS: rotate {math.degrees(delta):+.1f} deg')

        pos = self.align @ self.src_to_ned @ np.array([p.x, p.y, p.z]) + self.offset_ned
        r_ned_frd = self.align @ r_ned_frd

        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        vel = FLU_TO_FRD @ np.array([lin.x, lin.y, lin.z])
        rates = FLU_TO_FRD @ np.array([ang.x, ang.y, ang.z])

        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self.master.mav.odometry_send(
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

