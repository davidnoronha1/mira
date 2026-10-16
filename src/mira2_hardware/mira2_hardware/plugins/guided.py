"""GUIDED position/yaw targets and direct-line motion, using navigation feedback."""
import math
import subprocess
import time

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # Registers PoseStamped transforms.
from pymavlink import mavutil
from utils.navigation import quat_to_rot

from mira2_hardware.controller_plugin import ControllerPlugin

M = mavutil.mavlink
MASK_POS = 0b110111111000
MASK_POS_YAW = 0b100111111000
MASK_POS_VEL_YAW = 0b100111000000
SENDS = 3
DIRECT_RATE = 20.0
DIRECT_LEAD = 1.0
GUIDED = 4


class GuidedControl(ControllerPlugin):
    requires = ('navigation',)

    def __init__(self, controller):
        super().__init__(controller)
        self.args = self.options
        self.navigation = controller.plugins['navigation']
        self.simulation = self.options.simulation
        self.visualize = self.options.visualize
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, controller)
        self.pub_goal = self.create_publisher(PoseStamped, '/vision/goal', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(PoseStamped, '/goal_pose_direct', self._on_direct, 10)
        self.create_service(Trigger, '/mira/guidance/hold', self._hold)
        self.goal = self.direct = None
        self.goal_reached = self.allow_arm = False
        self.travel_sends = self.heading_sends = 0
        self.last_resend = self.last_direct = 0.0
        self.was_guided = False

    def stop(self, reason):
        self.goal = self.direct = None
        self.allow_arm = False
        self.travel_sends = self.heading_sends = 0

    def on_connect(self):
        self.stop('Autopilot connected')
        self.was_guided = self.mode == GUIDED

    def on_message(self, msg):
        if msg.get_type() == 'HEARTBEAT':
            if self.was_guided and self.mode != GUIDED:
                self.stop('Left GUIDED mode')
            self.was_guided = self.mode == GUIDED
            return
        if msg.get_type() != 'LOCAL_POSITION_NED' or self.goal is None or self.goal_reached:
            return
        if self.navigation.attitude is None:
            return
        distance = math.dist((msg.x, msg.y, msg.z), self.goal[:3])
        if distance < self.args.radius:
            self.goal_reached = True
            self.heading_sends = SENDS
            self.get_logger().info(f'Goal reached (EKF says {distance:.2f} m away)')

    def tick(self, now):
        if now - self.last_resend >= 0.3:
            self._resend_goal()
            self.last_resend = now
        if now - self.last_direct >= 1.0 / DIRECT_RATE - 1e-9:
            self._direct_step()
            # Keep the 20 Hz phase on the controller's 50 Hz clock (alternating
            # 40/60 ms callbacks), rather than drifting to one step every 60 ms.
            if now - self.last_direct >= 2.0 / DIRECT_RATE:
                self.last_direct = now
            else:
                self.last_direct += 1.0 / DIRECT_RATE

    # ---- ROS -> ArduSub -------------------------------------------------
    def _parse_goal(self, msg):
        """Return (msg in odom, depth, yaw_enu, yaw_ned) or None for an unusable goal."""
        if not self.ready or self.emergency_active or not msg.header.frame_id:
            return None
        try:
            if msg.header.frame_id != 'odom':
                msg = self.tf_buffer.transform(msg, 'odom')
        except Exception as exc:
            self.get_logger().warn(f'Goal transform unavailable: {exc}')
            return None
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return None
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01:
            return None
        # PoseStamped is metric 3D; z=0 is a real surface goal, never a hidden depth override.
        r_enu_flu = quat_to_rot(q.x, q.y, q.z, q.w)
        yaw_enu = math.atan2(r_enu_flu[1, 0], r_enu_flu[0, 0])
        yaw_ned = math.atan2(math.cos(yaw_enu), math.sin(yaw_enu))  # pi/2 - yaw_enu, wrapped
        return msg, -p.z, yaw_enu, yaw_ned

    def _on_direct(self, msg):
        parsed = self._parse_goal(msg)
        if parsed is None:
            return
        if self.navigation.latest_pose is None or time.monotonic() - self.navigation.last_feedback > 1.0 or not self.arm_state:
            self.get_logger().warn('Direct goal rejected: needs fresh EKF feedback and an armed vehicle')
            return
        msg, depth, _, yaw_ned = parsed
        start = self.navigation.latest_pose.pose.position
        start = (start.y, start.x, -start.z)  # NED
        end = (msg.pose.position.y, msg.pose.position.x, depth)
        delta = [b - a for a, b in zip(start, end)]
        length = math.sqrt(sum(d*d for d in delta))
        if length < 0.1:
            return
        direction = [d / length for d in delta]
        # Continue at the current speed along the line instead of restarting from rest.
        speed = max(0.0, sum(v*d for v, d in zip(self.navigation.velocity, direction)))
        self.goal = None
        self.travel_sends = self.heading_sends = 0
        self.direct = dict(start=start, end=end, direction=direction, length=length,
                           s=0.0, speed=min(speed, self.args.direct_speed), yaw=yaw_ned)
        self.get_logger().info(f'Direct line: {length:.2f} m at up to {self.args.direct_speed:.2f} m/s, '
                               f'no obstacle avoidance, heading {round(math.degrees(yaw_ned)) % 360} deg')

    def _direct_step(self):
        """Stream a carrot moving along the direct line with speed feedforward."""
        d = self.direct
        if d is None:
            return
        if self.mode != GUIDED:
            self.stop('Left GUIDED mode')
            return
        if (self.emergency_active or time.monotonic() - self.navigation.last_feedback > 1.0 or
                time.monotonic() - self.last_heartbeat > 3.0 or not self.arm_state):
            self.direct = None
            self.get_logger().error('Direct line aborted: feedback, arming or mode lost')
            if not self.emergency_active:
                self.request_arm(False)
            return
        dt = 1.0 / DIRECT_RATE
        remaining = d['length'] - d['s']
        # Trapezoidal speed: accelerate, cruise, and brake to stop exactly at the end.
        d['speed'] = min(self.args.direct_speed, d['speed'] + self.args.direct_accel * dt,
                         math.sqrt(2 * self.args.direct_accel * max(remaining, 0.0)))
        p = self.navigation.latest_pose.pose.position
        along = sum((c - a) * u for c, a, u in zip((p.y, p.x, -p.z), d['start'], d['direction']))
        d['s'] = min(d['length'], d['s'] + d['speed'] * dt, max(d['s'], along + DIRECT_LEAD))
        n, e, z = (a + u * d['s'] for a, u in zip(d['start'], d['direction']))
        vn, ve, vz = (u * d['speed'] for u in d['direction'])
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component, M.MAV_FRAME_LOCAL_NED,
            MASK_POS_VEL_YAW, n, e, z, vn, ve, vz, 0, 0, 0, d['yaw'], 0)
        if d['s'] >= d['length'] and d['speed'] < 1e-3:
            # Line complete: ArduSub holds the end point (zero velocity) from here.
            self.direct = None
            self.goal = (*d['end'], d['yaw'])
            self.goal_reached = True
            self.get_logger().info('Direct line complete; holding its end point')

    def _on_goal(self, msg):
        parsed = self._parse_goal(msg)
        if parsed is None:
            return
        msg, depth, yaw_enu, yaw_ned = parsed
        p = msg.pose.position
        self.direct = None
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
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component, M.MAV_FRAME_LOCAL_NED,
            MASK_POS_YAW if with_yaw else MASK_POS, n, e, d, 0, 0, 0, 0, 0, 0, yaw, 0)

    def _hold(self, request, response):
        self.allow_arm = False
        self.direct = None
        self.goal = None
        self.travel_sends = self.heading_sends = 0
        if self.navigation.latest_pose is None or time.monotonic() - self.navigation.last_feedback > 1.0:
            response.success = False
            response.message = 'No fresh EKF pose; disarm requested'
            self.request_arm(False)
            return response
        p = self.navigation.latest_pose.pose.position
        # Hold the current heading too: targets now always carry yaw.
        self.goal = (p.y, p.x, -p.z, self.navigation.attitude[2] if self.navigation.attitude else 0.0)
        self.goal_reached = True
        self.travel_sends = SENDS
        response.success = True
        response.message = 'Hold at current EKF position requested'
        return response

    def _resend_goal(self):
        # Bounded retries: continuously resending restarts ArduSub's OA path.
        if self.goal is None or self.emergency_active:
            return
        if (time.monotonic() - self.navigation.last_feedback > 1.0 or
                time.monotonic() - self.last_heartbeat > 3.0):
            self.goal = None
            self.request_arm(False)
            self.get_logger().error('GUIDED feedback lost; disarm requested')
            return
        if self.mode != GUIDED:
            self.request_mode(GUIDED)
            return
        if not self.arm_state:
            if self.simulation and self.args.auto_arm and self.allow_arm:
                self.request_arm(True)
            return
        if self.travel_sends > 0:
            # Position and heading together: ArduSub slews smoothly to the goal
            # heading while travelling, and OA only bends the position. Facing
            # along the path instead swung the nose with every BendyRuler bearing
            # change and needed a separate turn on arrival.
            self._send_target(with_yaw=True)
            self.travel_sends -= 1
        elif self.goal_reached and self.heading_sends > 0:
            self._send_target(with_yaw=True)
            self.heading_sends -= 1

