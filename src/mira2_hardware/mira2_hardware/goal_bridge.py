#!/usr/bin/env python3
"""Forward metric ROS ENU goals to Pixhawk GUIDED and publish EKF pose/TF.

All goal heights are literal, including z=0. The vision consumer aligns
external odometry to the local navigation frame. Only explicit simulation
mode enables Gazebo markers, ground-truth paths and optional auto-arming.
"""
import argparse
import math
import os
import subprocess
import time

os.environ.setdefault('MAVLINK20', '1')

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from pymavlink import mavutil
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformBroadcaster

from utils.navigation import ENU_TO_NED, FLU_TO_FRD, quat_to_rot, rot_to_quat_wxyz
from mira2_hardware.path_history import PathHistory

M = mavutil.mavlink
MASK_POS = 0b110111111000      # position only: ArduSub faces along its path (WP_YAW_BEHAVIOR)
MASK_POS_YAW = 0b100111111000  # position + yaw, ignore velocity/accel/yaw rate
SENDS = 3                      # copies of each target (UDP), then leave ArduSub alone
GUIDED = 4                       # ArduSub custom mode
PATH_STEP = 0.05                 # m between recorded path points


def euler_to_rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


class GoalBridge(Node):
    def __init__(self, args):
        super().__init__('goal_bridge')
        self.args = args
        self.simulation = args.simulation == 'true'
        if not self.simulation:
            args.auto_arm = 'false'
            if self.get_parameter('use_sim_time').value:
                raise ValueError('use_sim_time requires --simulation true')
        self.latest_pose = None
        self.last_feedback = 0.0
        self.last_heartbeat = 0.0
        self.emergency_active = False
        self.allow_arm = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, "/mira/guidance/hold", self._hold)
        self.create_service(SetBool, "/mira/emergency_stop", self._emergency)
        self.goal = None  # (n, e, d, yaw_ned)
        self.goal_reached = False
        self.travel_sends = 0   # position-only targets still to send for this goal
        self.heading_sends = 0  # final-heading targets still to send after arrival
        self.attitude = None
        self.armed = False
        self.mode = None
        self.visualize = args.visualize == 'true'
        self.ekf_path = PathHistory(args.path_max_points, PATH_STEP)
        self.gt_path = PathHistory(args.path_max_points, PATH_STEP)

        self.pub_pose = self.create_publisher(PoseStamped, '/vision/ekf_pose', 10)
        self.pub_ekf_path = self.create_publisher(Path, '/vision/ekf_path', 1)
        self.pub_gt_path = self.create_publisher(Path, '/vision/gt_path', 1) if self.simulation else None
        self.pub_goal = self.create_publisher(PoseStamped, '/vision/goal', 10)
        self.tf = TransformBroadcaster(self)  # odom -> base_link from ArduSub's estimate
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        if self.visualize and self.simulation:
            self.create_subscription(Odometry, '/vision/gt_odom', self._on_gt, 1)
        if self.visualize:
            self.create_timer(1.0 / args.path_rate, self._publish_paths)

        self.mav = mavutil.mavlink_connection(args.mavlink, source_system=253)
        self.get_logger().info(f'Waiting for ArduSub heartbeat on {args.mavlink} ...')
        while True:
            hb = self.mav.recv_match(type='HEARTBEAT', blocking=True)
            if hb.autopilot != M.MAV_AUTOPILOT_INVALID:
                self.mav.target_system, self.mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        # 30 Hz so the AUV moves smoothly in RViz (10 Hz looked jerky)
        for msg_id in (M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, M.MAVLINK_MSG_ID_ATTITUDE):
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                           M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 1e6 / 30, 0, 0, 0, 0, 0)
        self.get_logger().info('Connected. Click "2D Goal Pose" in RViz to send the vehicle somewhere.')

        self.create_timer(0.02, self._mav_rx)
        self.create_timer(0.3, self._resend_goal)

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
    def _mav_rx(self):
        for _ in range(64):
            msg = self.mav.recv_match(type=['HEARTBEAT', 'ATTITUDE', 'LOCAL_POSITION_NED', 'STATUSTEXT'],
                                      blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == 'HEARTBEAT':
                if msg.autopilot != M.MAV_AUTOPILOT_INVALID:
                    self.last_heartbeat = time.monotonic()
                    self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = msg.custom_mode
            elif t == 'ATTITUDE':
                self.attitude = (msg.roll, msg.pitch, msg.yaw)
            elif t == 'STATUSTEXT':
                self.get_logger().info(f'ArduSub: {msg.text}')
            elif self.attitude is not None:
                self._publish_ekf(msg)

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

        if self.goal and not self.goal_reached:
            n, e, d, _ = self.goal
            dist = math.dist((pos.x, pos.y, pos.z), (n, e, d))
            if dist < self.args.radius:
                self.goal_reached = True
                self.heading_sends = SENDS
                self.get_logger().info(f'Goal reached (EKF says {dist:.2f} m away); turning to the goal heading')
            else:
                self.get_logger().info(f'{dist:.2f} m to goal', throttle_duration_sec=2.0)

    def _on_gt(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        self.gt_path.append(ps)

    # ---- ROS -> ArduSub -------------------------------------------------
    def _on_goal(self, msg):
        if self.emergency_active or not msg.header.frame_id:
            return
        try:
            if msg.header.frame_id != 'odom':
                msg = self.tf_buffer.transform(msg, 'odom')
        except Exception as exc:
            self.get_logger().warn(f'Goal transform unavailable: {exc}')
            return
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01:
            return
        # PoseStamped is metric 3D; z=0 is a real surface goal, never a hidden depth override.
        depth = -p.z
        r_enu_flu = quat_to_rot(q.x, q.y, q.z, q.w)
        yaw_enu = math.atan2(r_enu_flu[1, 0], r_enu_flu[0, 0])
        yaw_ned = math.atan2(math.cos(yaw_enu), math.sin(yaw_enu))  # pi/2 - yaw_enu, wrapped
        self.allow_arm = True
        self.goal = (p.y, p.x, depth, yaw_ned)
        self.goal_reached = False
        self.travel_sends = SENDS
        self.heading_sends = 0
        self.get_logger().info(
            f'Goal: north {p.y:+.2f} m, east {p.x:+.2f} m, depth {depth:.2f} m, '
            f'heading {round(math.degrees(yaw_ned)) % 360} deg')

        echo = PoseStamped()
        echo.header.frame_id = 'odom'
        echo.header.stamp = self.get_clock().now().to_msg()
        echo.pose = msg.pose
        echo.pose.position.z = -depth
        self.pub_goal.publish(echo)

        if self.visualize and self.simulation:
            self._gz_goal_markers(p.x, p.y, -depth, yaw_enu)

        self._resend_goal()

    def _gz_goal_markers(self, x, y, z, yaw_enu):
        """Highlight the goal in the Gazebo GUI via its /marker service.

        GUI-only visibility, so the simulated cameras (and thus VO) never see
        it. Silently does nothing when Gazebo runs without a GUI.
        """
        if not self.simulation:
            return
        hx, hy = x + 0.6 * math.cos(yaw_enu), y + 0.6 * math.sin(yaw_enu)
        mat = 'material {{ ambient {{ {c} }} diffuse {{ {c} }} emissive {{ {c} }} }}'
        magenta = mat.format(c='r: 1 g: 0 b: 1 a: 0.9')
        markers = [
            f'id: 1 type: SPHERE pose {{ position {{ x: {x} y: {y} z: {z} }} }} '
            f'scale {{ x: 0.35 y: 0.35 z: 0.35 }} {magenta}',
            # pole up to the surface so the goal is easy to spot from above
            f'id: 2 type: CYLINDER pose {{ position {{ x: {x} y: {y} z: {z / 2} }} }} '
            f'scale {{ x: 0.04 y: 0.04 z: {max(-z, 0.05)} }} {magenta}',
            f'id: 3 type: LINE_LIST point {{ x: {x} y: {y} z: {z} }} point {{ x: {hx} y: {hy} z: {z} }} '
            f'scale {{ x: 0.05 y: 0.05 z: 0.05 }} {magenta}',
            # (no TEXT marker: the ogre2 GUI renderer rejects that type)
        ]
        for body in markers:
            req = f'ns: "goal" action: ADD_MODIFY visibility: GUI {body}'
            subprocess.Popen(['gz', 'service', '-s', '/marker', '--reqtype', 'gz.msgs.Marker',
                              '--reptype', 'gz.msgs.Empty', '--timeout', '1000', '--req', req],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _send_target(self, with_yaw):
        n, e, d, yaw = self.goal
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component, M.MAV_FRAME_LOCAL_NED,
            MASK_POS_YAW if with_yaw else MASK_POS, n, e, d, 0, 0, 0, 0, 0, 0, yaw, 0)

    def _hold(self, request, response):
        self.allow_arm = False
        self.goal = None
        self.travel_sends = self.heading_sends = 0
        if self.latest_pose is None or time.monotonic() - self.last_feedback > 1.0:
            response.success = False
            response.message = 'No fresh EKF pose; disarm requested'
            self.mav.arducopter_disarm()
            return response
        p = self.latest_pose.pose.position
        self.goal = (p.y, p.x, -p.z, 0.0)
        self.goal_reached = True
        self.travel_sends = SENDS
        response.success = True
        response.message = 'Hold at current EKF position requested'
        return response

    def _emergency(self, request, response):
        self.emergency_active = request.data
        self.goal = None
        self.travel_sends = self.heading_sends = 0
        self.mav.arducopter_disarm()
        response.success = True
        response.message = 'Emergency engaged' if request.data else 'Emergency cleared; send a new goal'
        return response

    def _resend_goal(self):
        # Bounded retries: continuously resending restarts ArduSub's OA path.
        if self.goal is None or self.emergency_active:
            return
        if (time.monotonic() - self.last_feedback > 1.0 or
                time.monotonic() - self.last_heartbeat > 3.0):
            self.goal = None
            self.mav.arducopter_disarm()
            self.get_logger().error('GUIDED feedback lost; disarm requested')
            return
        if self.mode != GUIDED:
            self.mav.set_mode_apm(GUIDED)
            return
        if not self.armed:
            if self.simulation and self.args.auto_arm == 'true' and self.allow_arm:
                self.mav.arducopter_arm()
            return
        if self.travel_sends > 0:
            # position only: ArduSub faces along its (OA-bent) path
            self._send_target(with_yaw=False)
            self.travel_sends -= 1
        elif self.goal_reached and self.heading_sends > 0:
            self._send_target(with_yaw=True)
            self.heading_sends -= 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mavlink', default='')
    ap.add_argument('--simulation', choices=('true', 'false'), default='false')
    ap.add_argument('--auto-arm', choices=('true', 'false'), default='false', help='arm on a goal (SITL opt-in)')
    ap.add_argument('--radius', type=float, default=0.3, help='goal reached radius (m)')
    ap.add_argument('--visualize', choices=('true', 'false'), default='true',
                    help='enable display paths and Gazebo goal markers (control/TF remain active)')
    ap.add_argument('--path-max-points', type=int, default=2000,
                    help='maximum recent points per displayed path')
    ap.add_argument('--path-rate', type=float, default=2.0,
                    help='display path publication rate in Hz (pose/TF rate is unchanged)')
    args, ros_args = ap.parse_known_args()
    if not args.mavlink:
        if args.simulation == 'true':
            args.mavlink = 'udpin:0.0.0.0:14557'
        else:
            ap.error('--mavlink is required outside simulation')
    if args.path_max_points < 1 or not math.isfinite(args.path_rate) or args.path_rate <= 0:
        ap.error('--path-max-points and --path-rate must be positive and finite')

    rclpy.init(args=['goal_bridge', *ros_args])
    node = GoalBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mav.arducopter_disarm()
        node.mav.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
