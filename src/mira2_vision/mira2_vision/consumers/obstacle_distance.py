"""Depth-event consumer producing body-frame proximity for Pixhawk and RViz."""
import math
from types import SimpleNamespace
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from utils.navigation import quat_to_rot
from utils.parameters import declare_config, Setting, positive
from mira2_vision.plugin import Plugin
from mira2_vision.consumers.mavlink import SensorLink, mavutil

M = mavutil.mavlink
SECTORS, STRIDE = 72, 2


class ObstacleDistance(Plugin):
    def __init__(self, node):
        super().__init__(node)
        if not self.settings['depth_topic']:
            raise ValueError('obstacle_distance requires a depth_topic')
        self.args = SimpleNamespace(**declare_config(node, {
            'mavlink': 'udpin:0.0.0.0:14558' if self.settings['simulation'] else '',
            'band': Setting(0.25, positive), 'min_range': Setting(0.2, positive),
            'max_range': Setting(8.0, positive),
        }, namespace='obstacles'))
        if not self.args.min_range < self.args.max_range < 655.34:
            raise ValueError('Obstacle range must satisfy 0 < min < max < 655.34 m')
        self.geom = self.geometry_key = self.info = None
        self.sent = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.pub_scan = node.create_publisher(LaserScan, 'obstacles', 1)
        self.link = SensorLink(node, self.args.mavlink, M.MAV_COMP_ID_OBSTACLE_AVOIDANCE)
        self.mav = self.link.connection

    def close(self):
        self.tf_listener.unregister()
        self.link.close()

    def _setup(self, frame_id):
        """Precompute per-pixel body-frame rays and sector bins (once)."""
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame_id, Time())
        except Exception as e:  # noqa: BLE001 - tf2 raises several types
            self.node.get_logger().warn(f'waiting for TF base_link <- {frame_id}: {e}', throttle_duration_sec=5.0)
            return False
        q, t = tf.transform.rotation, tf.transform.translation
        rot = quat_to_rot(q.x, q.y, q.z, q.w)  # optical -> base_link (FLU)

        k = self.info
        fx, cx, fy, cy = k.k[0], k.k[2], k.k[4], k.k[5]
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            return False
        u, v = np.meshgrid(np.arange(0, k.width, STRIDE), np.arange(0, k.height, STRIDE))
        rays_opt = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=float)], axis=-1)
        rays_body = rays_opt @ rot.T  # per unit of optical depth
        hfov = 2 * math.atan2(k.width / 2, fx)
        self.geom = dict(rays=rays_body, origin=np.array([t.x, t.y, t.z]), hfov=hfov)
        self.node.get_logger().info(
            f'depth camera {k.width}x{k.height}, HFOV {math.degrees(hfov):.1f} deg, mount pitch '
            f'{math.degrees(math.asin(-rot[2, 2])):.1f} deg down; obstacles = points within '
            f'+/-{self.args.band} m of the AUV height')
        return True

    def process(self, frame):
        pass  # Obstacle updates run at depth rate, independently of RGB or YOLO.

    def on_depth(self, sample):
        info = sample.info
        if (info is None or info.header.frame_id != sample.header.frame_id or
                (info.height, info.width) != sample.image.shape):
            return
        key = (info.width, info.height, tuple(info.k), sample.header.frame_id)
        if key != self.geometry_key:
            self.geom = None
            self.geometry_key = key
        self.info = info
        if self.geom is None and not self._setup(sample.header.frame_id):
            return
        g = self.geom
        depth = sample.image[::STRIDE, ::STRIDE]
        msg = sample.message
        valid = np.isfinite(depth) & (depth > 0.05)
        if not valid.any():
            return  # Missing depth is not a clear path.
        pts = g['rays'] * np.where(valid, depth, 0)[..., None] + g['origin']   # base_link FLU
        in_band = valid & (np.abs(pts[..., 2]) <= self.args.band)
        x, y = pts[..., 0][in_band], pts[..., 1][in_band]
        rng = np.hypot(x, y)
        bearing = np.arctan2(-y, x)  # clockwise positive (MAVLink / FRD)

        half = g['hfov'] / 2
        keep = (np.abs(bearing) < half) & (rng >= self.args.min_range)
        sector_idx = ((bearing[keep] + half) / g['hfov'] * SECTORS).astype(int).clip(0, SECTORS - 1)
        sectors = np.full(SECTORS, np.inf)
        np.minimum.at(sectors, sector_idx, rng[keep])

        max_cm = int(self.args.max_range * 100)
        dist_cm = np.where(sectors <= self.args.max_range, np.round(sectors * 100), max_cm + 1).astype(int)

        inc_deg = math.degrees(g['hfov']) / SECTORS
        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        if self.link.ready:
            self.mav.mav.obstacle_distance_send(
                stamp_us, M.MAV_DISTANCE_SENSOR_LASER, [int(d) for d in dist_cm], 0,
                int(self.args.min_range * 100), max_cm,
                # ArduPilot puts sector j at angle_offset + j*increment, so offset
                # to the centre of sector 0
                inc_deg, -math.degrees(half) + inc_deg / 2, M.MAV_FRAME_BODY_FRD)
            self.sent += 1

        # Same data for RViz: LaserScan is counter-clockwise from angle_min,
        # so reverse the clockwise MAVLink sector order.
        if not self.pub_scan.get_subscription_count():
            return  # MAVLink above is always sent, even without visualization.
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = 'base_link'
        scan.angle_increment = math.radians(inc_deg)
        scan.angle_min = -half + scan.angle_increment / 2
        scan.angle_max = scan.angle_min + scan.angle_increment * (SECTORS - 1)
        scan.range_min = self.args.min_range
        scan.range_max = self.args.max_range
        scan.ranges = [float(r) if r <= self.args.max_range else float('inf') for r in sectors[::-1]]
        self.pub_scan.publish(scan)

