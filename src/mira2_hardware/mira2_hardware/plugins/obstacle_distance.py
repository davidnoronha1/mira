"""Forward body-frame ROS LaserScan measurements as MAVLink obstacle distances."""
import math

from pymavlink import mavutil
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from utils.parameters import Setting, declare_config, positive
from utils import time as utime

from mira2_hardware.controller_plugin import ControllerPlugin


class ObstacleDistance(ControllerPlugin):
    def __init__(self, controller):
        super().__init__(controller)
        self.config = declare_config(controller, {
            'topic': '/mira/perception/front/obstacles',
            'max_age': Setting(0.5, positive),
        }, namespace='obstacle_distance')
        self.create_subscription(LaserScan, self.config['topic'], self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        if not self.ready or msg.header.frame_id != 'base_link':
            return
        if not -0.05 <= utime.stamp_age(self.get_clock(), msg.header.stamp) <= self.config['max_age']:
            return
        count = len(msg.ranges)
        if (not 1 <= count <= 72 or not math.isfinite(msg.angle_min)
                or not math.isfinite(msg.angle_increment) or msg.angle_increment <= 0
                or not 0 < msg.range_min < msg.range_max < 655.34):
            return
        distances = []
        for value in reversed(msg.ranges):  # ROS CCW -> MAVLink clockwise.
            if math.isnan(value) or value < msg.range_min:
                distances.append(65535)  # Unknown, never a fabricated clear path.
            elif value > msg.range_max:
                distances.append(int(msg.range_max * 100) + 1)
            else:
                distances.append(round(value * 100))
        distances.extend([65535] * (72 - count))
        last_angle = msg.angle_min + (count - 1) * msg.angle_increment
        self.master.mav.obstacle_distance_send(
            int(utime.stamp_seconds(msg.header.stamp) * 1e6),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, distances, 0,
            int(msg.range_min * 100), int(msg.range_max * 100),
            math.degrees(msg.angle_increment), -math.degrees(last_angle),
            mavutil.mavlink.MAV_FRAME_BODY_FRD)
