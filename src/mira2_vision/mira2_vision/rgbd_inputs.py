"""Shared synchronization and validation for registered RGB-D consumers."""
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from utils import topics
from utils.parameters import Setting, declare_config, positive
from utils import time as utime


class RGBDInputs(Node):
    def __init__(self, name):
        super().__init__(name)
        self.config = declare_config(self, {
            'rgb_topic': topics.FRONT_IMAGE,
            'depth_topic': topics.FRONT_DEPTH,
            'camera_info_topic': topics.FRONT_CAMERA_INFO,
            'max_depth': Setting(8.0, lambda v: positive(v) and v < 65.535),
            'max_skew': Setting(0.01, positive), 'max_age': Setting(0.5, positive),
        }, namespace='rgbd_inputs')
        self.bridge = CvBridge()
        self.info = None
        self.create_subscription(CameraInfo, self.config['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data)
        self.rgb = Subscriber(self, Image, self.config['rgb_topic'], qos_profile=qos_profile_sensor_data)
        self.depth = Subscriber(self, Image, self.config['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.rgb, self.depth], 10, self.config['max_skew'])
        self.sync.registerCallback(self.on_pair)

    def on_info(self, msg):
        self.info = msg

    def _valid_pair(self, rgb, depth):
        if self.info is None or not rgb.header.frame_id:
            return False
        if rgb.header.frame_id != depth.header.frame_id or rgb.header.frame_id != self.info.header.frame_id:
            return False  # Register depth to RGB before supplying it to SLAM.
        if (rgb.width, rgb.height) != (depth.width, depth.height) or (rgb.width, rgb.height) != (self.info.width, self.info.height):
            return False
        if not np.isfinite(self.info.k).all() or self.info.k[0] <= 0 or self.info.k[4] <= 0:
            return False
        return all(-0.05 <= utime.stamp_age(self.get_clock(), msg.header.stamp) <= self.config['max_age']
                   for msg in (rgb, depth))

    def on_pair(self, rgb, depth):
        """Process a synchronized pair in the consuming estimator."""
        raise NotImplementedError
