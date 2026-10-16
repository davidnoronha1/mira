"""Downward-camera optical flow: metric body velocity over a flat pool floor.

Floor features are tracked between consecutive frames (pyramidal Lucas-Kanade
with a forward-backward check). Image motion caused by the vehicle rotating is
removed with the autopilot's raw gyro integrated between the frames, and the
zoom caused by a depth change with the depth sensor. Rotation from EKF attitude
changes is NOT used: an EKF gyro-bias error then shows up as flow velocity,
which the EKF fuses into a larger error - at boot this loop diverged on about
two of three starts (tilt 20 deg, velocity 30 m/s while stationary). The EKF
attitude is used only for the absolute camera tilt. Attitude comes from /mira/state/attitude, which the hardware navigation plugin
publishes even before the EKF has any horizontal position. Height comes from
the raw depth sensor (/mira/state/depth, low-pass filtered), not the EKF:
in a recorded flow-only run the EKF depth scattered 0.27 m (0.67 m worst) and
made the flow 0.23 m/s RMS wrong, versus 0.03 m/s with true height. The
remaining translational flow of each feature is scaled to
metres with that floor point's distance from the camera; the median is used.

There is no range sensor: height above the floor = floor_depth - camera depth,
with the camera depth from the depth sensor. floor_depth is therefore the
floor's depth as the vehicle's pressure sensor reads it; calibrate it per pool
by resting on the bottom (required, no default). SCENE ASSUMPTION: the floor is
flat at that depth everywhere the vehicle flies.

Features are spread over a grid of cells on a contrast-equalized (CLAHE) image.
A plain strongest-first pick clusters on the few high-contrast floor stripes
and leaves the faint tile lines across them unused; points on a single line
only constrain motion across that line.

Floor-line heading (opt-in: floor_heading, off by default). SCENE ASSUMPTION:
the floor's visible edges are straight lines along the pool axes (tile grout,
lane lines), so their dominant direction, projected onto the floor with the
EKF attitude, is the pool axis as the EKF currently sees it; the difference
from the known axis (line_heading, modulo line_symmetry) is the EKF's yaw
error. Validated only in the simulated SAUVC pool (mira_sim scene
sauvc_pool). A floor without such lines, or with dominant edges at other
angles (drains, shadows, mats, a patterned floor), gives a wrong heading that
the EKF will fuse. It is ambiguous modulo the symmetry (90 deg for a tiled
floor), which is harmless while the EKF yaw error stays well below half of it.
The corrected heading is published for the hardware external_navigation plugin to fuse as an
ExternalNav yaw; the autopilot must then use ExternalNav yaw in the flow
source set (EK3_SRC2_YAW 6, see mira2_hardware/config/ardusub).

Publishes geometry_msgs/TwistWithCovarianceStamped in base_link (FLU) and
PoseWithCovarianceStamped (odom ENU, orientation only) for the heading. The
hardware external_navigation plugin forwards both to the autopilot; this plugin owns no MAVLink link.
"""
from collections import deque
import math

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.time import Time
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation, Slerp
from tf2_ros import Buffer, TransformListener

from mira2_vision.plugin import Plugin
from utils import topics
from utils.parameters import choices, declare_config, Setting, nonnegative, positive

MIN_TRACKS = 25         # fewer usable floor features: no measurement this frame
MAX_FB_ERROR = 1.0      # px, forward-backward consistency (at processing scale)
MIN_HEIGHT = 0.2        # m above the floor; closer and the scale is meaningless
MAX_DT = 0.25           # s between frames; longer gaps restart tracking
MIN_COSINE = 0.2        # rays this oblique to the floor are not used
PRESSURE_TAU = 0.5      # s low-pass on pressure depth (SITL sample noise ~0.07 m)
MIN_SIGMA = 0.05        # m/s; floor on the reported velocity uncertainty
MAX_GYRO_GAP = 0.05     # s; gyro arrives at ~100 Hz, larger gaps skip the frame
# While the depth changes fast the floor image zooms, and the filtered depth
# lags, so residual radial flow leaks into the median as false sideways motion
# (0.56 m drift in the starting ring during the descent). Skip those frames.
MAX_VERTICAL_SPEED = 0.15  # m/s
REDETECT_BELOW = 80     # keep tracking surviving features; re-detect when fewer
GRID = (4, 5)           # feature cells (rows, cols); each gets an equal share
# Floor-line heading. Offline (1192 sim frames, yaw errors up to 34 deg injected)
# the recovered heading was 1.8 deg RMS. Strength is the weighted alignment of
# the floor edges with the line pattern (1 = all edges on the pool axes).
MIN_LINE_STRENGTH = 0.3
MIN_LINE_PIXELS = 200
MAX_LINE_PIXELS = 4000
MIN_HEADING_SIGMA = math.radians(5.0)  # EKF3 floors ExternalNav yaw error at 5 deg anyway


class OpticalFlow(Plugin):
    def __init__(self, node):
        super().__init__(node)
        self.args = declare_config(node, {
            # Per pool (see module doc); configs/scene files set it, 0 fails startup.
            'floor_depth': Setting(0.0, nonnegative),
            'topic': topics.FLOW_VELOCITY,
            'attitude_topic': topics.ATTITUDE,
            'pressure_topic': topics.DEPTH,
            'height_source': Setting('pressure', choices('pressure', 'ekf')),
            'base_frame': 'base_link',
            'scale': Setting(0.5, positive),  # image downscale for tracking
            'max_features': Setting(200, positive),
            # Faster than the vehicle can move: a tracking failure, not motion.
            'max_speed': Setting(2.0, positive),
            # Floor-line heading, SCENE-SPECIFIC (see module doc). Off unless the
            # pool is known to have straight floor lines along its axes.
            'floor_heading': False,
            'heading_topic': topics.FLOOR_HEADING,
            # Raw autopilot IMU (sensor_msgs/Imu, base_link FLU) for derotation.
            # '' falls back to EKF attitude differences (see module doc).
            'gyro_topic': topics.STATE_IMU,
            # SCENE-SPECIFIC: direction of the pool's lines in odom (ENU, deg
            # from +x, mod line_symmetry). Surveyed per pool, or
            # line_heading_from_start: the first measurement defines it
            # (trusts the startup heading, e.g. the compass).
            'line_heading': 0.0,
            'line_heading_from_start': True,
            'line_symmetry': Setting(90.0, choices(90.0, 180.0)),  # tiles: 90, lane lines only: 180
        }, namespace='optical_flow')
        if self.args['floor_depth'] <= 0:
            raise ValueError('optical_flow.floor_depth (pool floor pressure depth in m) must be configured '
                             '(mira2_vision/config/optical_flow.yaml, or the simulator scene file)')
        self.publisher = node.create_publisher(TwistWithCovarianceStamped, self.args['topic'], 10)
        self.heading_publisher = (node.create_publisher(PoseWithCovarianceStamped, self.args['heading_topic'], 10)
                                  if self.args['floor_heading'] and self.args['heading_topic'] else None)
        self.line_heading = (None if self.args['line_heading_from_start']
                             else math.radians(self.args['line_heading']))
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.previous = None  # (stamp_s, gray, points, camera rotation in odom, camera z)
        self.base_from_camera = self.camera_offset = None
        self.attitudes = deque(maxlen=200)  # (stamp_s, base rotation in odom, base z)
        self.attitude_sub = node.create_subscription(PoseStamped, self.args['attitude_topic'], self._on_attitude, 50)
        self.pressure_z = self.pressure_stamp = None  # filtered base_link z from the depth sensor
        self.pressure_rate = 0.0  # filtered vertical speed, m/s
        self.pressure_sub = node.create_subscription(PointStamped, self.args['pressure_topic'], self._on_pressure, 50)
        self.gyro = deque(maxlen=400)  # (stamp_s, angular velocity FLU rad/s)
        self.gyro_sub = (node.create_subscription(Imu, self.args['gyro_topic'], self._on_gyro, 200)
                         if self.args['gyro_topic'] else None)
        self.published = self.skipped = self.headings = 0
        self.heading_error = []  # applied corrections, for the periodic report
        self.timer = node.create_timer(10.0, self._report)

    def close(self):
        self.node.destroy_timer(self.timer)
        self.node.destroy_subscription(self.attitude_sub)
        self.node.destroy_subscription(self.pressure_sub)
        if self.gyro_sub:
            self.node.destroy_subscription(self.gyro_sub)
        self.tf_listener.unregister()
        self.node.destroy_publisher(self.publisher)
        if self.heading_publisher:
            self.node.destroy_publisher(self.heading_publisher)

    def _report(self):
        drift = (f', EKF yaw error {np.degrees(np.mean(self.heading_error)):+.1f} deg mean '
                 f'(max {np.degrees(np.max(np.abs(self.heading_error))):.1f})' if self.heading_error else '')
        self.node.get_logger().info(f'optical flow: {self.published} velocities, {self.skipped} skipped, '
                                    f'{self.headings} floor headings in 10 s{drift}')
        self.published = self.skipped = self.headings = 0
        self.heading_error = []

    def _on_attitude(self, message):
        q, z = message.pose.orientation, message.pose.position.z
        if math.isfinite(z) and math.isfinite(q.x + q.y + q.z + q.w):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.attitudes.append((stamp, Rotation.from_quat([q.x, q.y, q.z, q.w]), z))

    def _on_gyro(self, message):
        w = message.angular_velocity
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if math.isfinite(w.x + w.y + w.z):
            self.gyro.append((stamp, np.array([w.x, w.y, w.z])))

    def _body_rotation(self, start, end):
        """Body rotation from start to end (expressed in the start body frame), from the gyro.

        Each sample's rate holds until the next sample. None without samples
        covering the whole interval.
        """
        samples = [s for s in self.gyro if start - MAX_GYRO_GAP <= s[0] <= end]
        if not samples or samples[0][0] > start or end - samples[-1][0] > MAX_GYRO_GAP:
            return None
        rotation = Rotation.identity()
        for (t0, w), (t1, _) in zip(samples, samples[1:] + [(end, None)]):
            dt = min(t1, end) - max(t0, start)
            if dt > 0:
                if t1 - t0 > MAX_GYRO_GAP:
                    return None
                rotation = rotation * Rotation.from_rotvec(w * dt)
        return rotation

    def _on_pressure(self, message):
        z = message.point.z
        if not math.isfinite(z):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.pressure_z is None or not 0 < stamp - self.pressure_stamp < 1.0:
            self.pressure_z, self.pressure_rate = z, 0.0
        else:
            gain = min(1.0, (stamp - self.pressure_stamp) / PRESSURE_TAU)
            previous = self.pressure_z
            self.pressure_z += (z - self.pressure_z) * gain
            rate = (self.pressure_z - previous) / (stamp - self.pressure_stamp)
            self.pressure_rate += (rate - self.pressure_rate) * gain
        self.pressure_stamp = stamp

    def _camera_pose(self, stamp):
        """Camera rotation and z in odom, attitude interpolated to stamp.

        Nearest-sample attitude was up to ~50 ms off: 1.5 deg of heading during
        the 30 deg/s scan turn, which the floor-line heading would then report.
        """
        if not self.attitudes:
            return None
        after = next((i for i, a in enumerate(self.attitudes) if a[0] >= stamp), None)
        if after is None or after == 0:
            sample = self.attitudes[-1] if after is None else self.attitudes[0]
            if abs(sample[0] - stamp) > 0.2:
                return None
            _, base, z = sample
        else:
            (t0, r0, z0), (t1, r1, z1) = self.attitudes[after - 1], self.attitudes[after]
            if t1 - t0 > 0.2:
                return None
            f = (stamp - t0) / (t1 - t0) if t1 > t0 else 0.0
            base = Slerp([0.0, 1.0], Rotation.concatenate([r0, r1]))(f)
            z = z0 + (z1 - z0) * f
        if self.args['height_source'] == 'pressure':
            if self.pressure_z is None or abs(self.pressure_stamp - stamp) > 0.5:
                return None
            z = self.pressure_z
            if abs(self.pressure_rate) > MAX_VERTICAL_SPEED:
                return None
        return base * self.base_from_camera, z + base.apply(self.camera_offset)[2]

    def process(self, frame):
        if frame.matrix is None:
            return
        if self.base_from_camera is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.args['base_frame'], frame.header.frame_id, Time())
                q, t = tf.transform.rotation, tf.transform.translation
                self.base_from_camera = Rotation.from_quat([q.x, q.y, q.z, q.w])
                self.camera_offset = np.array([t.x, t.y, t.z])
            except Exception as error:  # noqa: BLE001
                self.node.get_logger().warn(f'optical flow waiting for TF: {error}', throttle_duration_sec=5.0)
                return
        stamp = frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9
        pose = self._camera_pose(stamp)
        if pose is None:
            # Also while descending/ascending fast; drop tracking state too.
            self.previous = None
            self.skipped += 1
            return
        s = self.args['scale']
        gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        matrix = frame.matrix.copy()
        matrix[:2] *= s
        if self.heading_publisher:
            self._publish_heading(frame.header, gray, matrix, *pose)
        gray = self.clahe.apply(gray)
        previous, self.previous = self.previous, None
        points = None
        if previous is not None and 0 < stamp - previous[0] <= MAX_DT and previous[2] is not None:
            points = self._measure(frame.header, stamp, gray, matrix, pose, previous)
        if points is None or len(points) < REDETECT_BELOW:
            points = self._detect(gray)
        self.previous = (stamp, gray, points, *pose)

    def _detect(self, gray):
        """Shi-Tomasi corners, an equal share per grid cell."""
        rows, cols = GRID
        h, w = gray.shape
        share = max(1, int(self.args['max_features']) // (rows * cols))
        found = []
        for r in range(rows):
            for c in range(cols):
                y0, x0 = r * h // rows, c * w // cols
                cell = cv2.goodFeaturesToTrack(gray[y0:(r + 1) * h // rows, x0:(c + 1) * w // cols], share, 0.02, 6)
                if cell is not None:
                    found.append(cell + np.array([x0, y0], np.float32))
        return np.concatenate(found) if found else None

    def _publish_heading(self, header, gray, matrix, camera_rotation, camera_z):
        """EKF heading corrected by the floor lines' direction (see module doc)."""
        height = self.args['floor_depth'] + camera_z
        if height < MIN_HEIGHT:
            return
        measured = self._line_axis(gray, matrix, camera_rotation, height)
        if measured is None:
            return
        axis, strength = measured
        period = math.radians(self.args['line_symmetry'])
        if self.line_heading is None:
            self.line_heading = axis
            self.node.get_logger().info(f'floor line heading set from the start pose: {math.degrees(axis):.1f} deg')
        correction = (self.line_heading - axis + period / 2) % period - period / 2
        self.heading_error.append(-correction)
        base = Rotation.from_euler('z', correction) * camera_rotation * self.base_from_camera.inv()
        message = PoseWithCovarianceStamped()
        message.header.stamp = header.stamp
        message.header.frame_id = 'odom'
        message.pose.pose.position.x = message.pose.pose.position.y = message.pose.pose.position.z = math.nan
        q = message.pose.pose.orientation
        q.x, q.y, q.z, q.w = (float(c) for c in base.as_quat())
        # Spread of the edge directions about the axis, reduced to the mean's.
        k = 2 * math.pi / period
        sigma = max(math.sqrt(-2 * math.log(strength)) / k / 4, MIN_HEADING_SIGMA)
        message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[14] = 1e6
        message.pose.covariance[21] = message.pose.covariance[28] = 1e6  # roll/pitch not measured
        message.pose.covariance[35] = sigma ** 2
        self.heading_publisher.publish(message)
        self.headings += 1

    def _line_axis(self, gray, matrix, camera_rotation, height):
        """Dominant floor-edge direction in odom (rad, mod line_symmetry) and its strength.

        Each strong edge pixel and a point 2 px along the edge are projected onto
        the floor; their direction votes with weight = gradient magnitude, on the
        angle multiplied by 360/symmetry so that parallel and perpendicular lines
        agree.
        """
        blurred = cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)
        gx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3)
        magnitude = np.hypot(gx, gy)
        magnitude[:4], magnitude[-4:], magnitude[:, :4], magnitude[:, -4:] = 0, 0, 0, 0
        ys, xs = np.nonzero(magnitude > max(np.percentile(magnitude, 90), 4.0))
        if len(xs) < MIN_LINE_PIXELS:
            return None
        if len(xs) > MAX_LINE_PIXELS:
            keep = np.linspace(0, len(xs) - 1, MAX_LINE_PIXELS).astype(int)
            ys, xs = ys[keep], xs[keep]
        weight = magnitude[ys, xs]
        tangent = np.column_stack((-gy[ys, xs], gx[ys, xs])) / weight[:, None]
        start = np.column_stack((xs, ys, np.ones(len(xs))))
        end = start + np.column_stack((2.0 * tangent, np.zeros(len(xs))))
        to_odom = (camera_rotation.as_matrix() @ np.linalg.inv(matrix)).T
        rays0, rays1 = start @ to_odom, end @ to_odom
        valid = (rays0[:, 2] < -MIN_COSINE) & (rays1[:, 2] < -MIN_COSINE)
        if valid.sum() < MIN_LINE_PIXELS:
            return None
        floor0 = rays0[valid, :2] * (height / -rays0[valid, 2])[:, None]
        floor1 = rays1[valid, :2] * (height / -rays1[valid, 2])[:, None]
        direction = floor1 - floor0
        k = 360.0 / self.args['line_symmetry']
        vote = np.sum(weight[valid] * np.exp(1j * k * np.arctan2(direction[:, 1], direction[:, 0])))
        strength = abs(vote) / np.sum(weight[valid])
        if strength < MIN_LINE_STRENGTH:
            return None
        return float(np.angle(vote) / k), float(strength)

    def _measure(self, header, stamp, gray, matrix, pose, previous):
        rotation, camera_z = pose
        last_stamp, last_gray, last_points, last_rotation, last_z = previous
        dt = stamp - last_stamp
        floor = self.args['floor_depth']
        height = floor + camera_z  # camera_z is negative below the surface
        if height < MIN_HEIGHT or floor + last_z < MIN_HEIGHT:
            self.skipped += 1
            return None
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(last_gray, gray, last_points, None)
        back, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, last_gray, tracked, None)
        good = ((status.ravel() == 1) & (back_status.ravel() == 1) &
                (np.linalg.norm((back - last_points).reshape(-1, 2), axis=1) < MAX_FB_ERROR))
        if good.sum() < MIN_TRACKS:
            self.skipped += 1
            return None
        survivors = tracked[good].reshape(-1, 1, 2)
        inverse = np.linalg.inv(matrix)
        homogeneous = lambda p: np.column_stack((p.reshape(-1, 2), np.ones(len(p.reshape(-1, 2)))))
        before = homogeneous(last_points[good]) @ inverse.T
        after = homogeneous(tracked[good]) @ inverse.T
        # Where each previous ray would appear after the rotation and the depth
        # change alone (the floor magnifies by h_prev/h as the camera descends).
        if self.gyro_sub:
            turned = self._body_rotation(last_stamp, stamp)
            if turned is None:
                self.skipped += 1
                return survivors
            relative = (self.base_from_camera.inv() * turned.inv() * self.base_from_camera).as_matrix()
        else:
            relative = (rotation.inv() * last_rotation).as_matrix()
        zoom = (floor + last_z) / height
        predicted = before @ (np.diag([zoom, zoom, 1.0]) @ relative).T
        predicted = predicted[:, :2] / predicted[:, 2:3]
        flow = after[:, :2] - predicted  # normalized image units, translation only
        # Each floor point's depth along the optical axis: the ray (x, y, 1)
        # reaches the floor `height` below the camera at Z = height / (down . ray).
        down = rotation.inv().apply([0.0, 0.0, -1.0])
        cosine = after @ down
        valid = cosine > MIN_COSINE
        if valid.sum() < MIN_TRACKS:
            self.skipped += 1
            return survivors
        # A floor point at depth Z moves by -t/Z for a lateral camera translation t.
        shifts = -flow[valid] * (height / cosine[valid])[:, None]
        median = np.median(shifts, axis=0)
        spread = 1.4826 * np.median(np.abs(shifts - median), axis=0)
        velocity = self.base_from_camera.apply(np.array([median[0], median[1], 0.0]) / dt)
        if np.linalg.norm(velocity) > self.args['max_speed']:
            self.skipped += 1
            return survivors
        sigma = max(float(np.max(spread)) / dt / math.sqrt(valid.sum()), MIN_SIGMA)
        message = TwistWithCovarianceStamped()
        message.header.stamp = header.stamp
        message.header.frame_id = self.args['base_frame']
        v = message.twist.twist.linear
        v.x, v.y, v.z = (float(c) for c in velocity)
        message.twist.covariance[0] = message.twist.covariance[7] = sigma ** 2
        message.twist.covariance[14] = 1e6  # vertical velocity is not observed
        message.twist.covariance[21] = message.twist.covariance[28] = message.twist.covariance[35] = 1e6
        self.publisher.publish(message)
        self.published += 1
        return survivors
