#!/usr/bin/env python3
"""One frame pipeline per camera: YOLO, ArUco, docking board and pipeline."""

from collections import deque
import json
from pathlib import Path
import re
import utils.time as utime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from utils.parameters import Setting, declare_config, positive, string_list
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from utils.image_source import build_image_source
from utils import topics
from mira2_vision.depth import DepthFrame, decode_depth, matches
from mira2_vision.plugin import Frame, load_plugin, order_plugins


class MiraVisionNode(Node):
    def __init__(self, **node_options):
        super().__init__("perception", **node_options)
        self.plugins = []
        self.source = None
        self.settings = self._load_settings()
        self._initialize_camera()
        self._load_plugins()
        self._create_interfaces()
        self._start_processing()

    def _load_settings(self):
        settings = declare_config(self, {
            'image_source': f'ros2://{topics.FRONT_IMAGE}',
            'camera_info_topic': topics.FRONT_CAMERA_INFO,
            'raw_image_topic': topics.FRONT_IMAGE,
            'camera_frame': 'camera_front_optical', 'calibration_file': '',
            'simulation': False, 'depth_camera_info_topic': '', 'depth_topic': '',
            'depth_max_skew': Setting(0.1, positive),
            'detectors': Setting(['yolo'], string_list, dynamic_typing=True),
            'consumers': Setting(['objects'], string_list, dynamic_typing=True),
            'rate': Setting(30.0, positive), 'max_frame_age': Setting(0.5, positive),
            'publish_image': True, 'visualize': False,
            'preview_rate': Setting(5.0, positive),
            'preview_width': Setting(640, lambda v: v >= 0), 'is_stereo_image': False,
        })
        if not settings['simulation'] and self.get_parameter('use_sim_time').value:
            raise ValueError('use_sim_time requires simulation=true')
        return settings

    def _initialize_camera(self):
        self.depth_frame = None
        self.depth_frames = deque(maxlen=8)
        self.depth_info = None
        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_size = None
        self.create_subscription(CameraInfo, self.settings["camera_info_topic"],
                                 self.camera_info_callback, qos_profile_sensor_data)
        if self.settings["calibration_file"]:
            self.load_calibration(self.resolve_file(self.settings["calibration_file"], "config"))

    def _load_plugins(self):
        specifications = self.settings['detectors'] + self.settings['consumers']
        if len(specifications) != len(set(specifications)):
            raise ValueError('A vision plugin may only be attached once')
        self.plugins = order_plugins([load_plugin(spec, self) for spec in specifications])

    def _create_interfaces(self):
        self.depth_pub = self.create_publisher(Image, 'depth', qos_profile_sensor_data)
        if self.settings['depth_topic']:
            self.create_subscription(Image, self.settings['depth_topic'], self.depth_callback,
                                     qos_profile_sensor_data)
        if self.settings['depth_camera_info_topic']:
            self.create_subscription(CameraInfo, self.settings['depth_camera_info_topic'],
                                     self.depth_info_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, "image", qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.raw_pub = None
        if not self.settings["image_source"].startswith("ros2://"):
            self.raw_pub = self.create_publisher(Image, self.settings["raw_image_topic"], qos_profile_sensor_data)

    def _start_processing(self):
        self.source = build_image_source(self.settings["image_source"], self)
        self.sequence = None
        self.last_frame = 0.0
        self.last_preview = 0.0
        self.processed = 0
        self.last_inference_ms = 0.0
        self.create_timer(utime.seconds_from_rate(self.settings["rate"]), self.process)
        self.create_timer(1.0, self.publish_diagnostics)

    @staticmethod
    def resolve_file(value, directory):
        if value.startswith("package://"):
            package, relative = value[len("package://"):].split("/", 1)
            return Path(get_package_share_directory(package)) / relative
        path = Path(value)
        if not path.is_absolute():
            candidate = Path(get_package_share_directory("mira2_vision")) / directory / path
            if candidate.exists():
                path = candidate
        if not path.is_file():
            raise FileNotFoundError(path)
        return path

    def load_calibration(self, path):
        if path.suffix == ".npz":
            with np.load(path) as data:
                self.camera_matrix = np.asarray(data["mtx"], np.float64)
                self.dist_coeffs = np.asarray(data["dist"], np.float64)
        elif path.suffix == ".json":
            data = json.loads(path.read_text())
            self.camera_matrix = np.asarray(data["camera_matrix"], np.float64)
            self.dist_coeffs = np.asarray(data["dist_coeff"], np.float64)
        elif path.suffix == ".ini":
            content = path.read_text()
            matrix = re.search(r"camera matrix\s*\n([\d\s.eE+\-]+)", content)
            distortion = re.search(r"distortion\s*\n([\d\s.eE+\-]+)", content)
            if not matrix or not distortion:
                raise ValueError(f"Invalid camera calibration: {path}")
            self.camera_matrix = np.fromstring(matrix.group(1), sep=" ").reshape(3, 3)
            self.dist_coeffs = np.fromstring(distortion.group(1), sep=" ")
            width = re.search(r"width\s*\n(\d+)", content)
            height = re.search(r"height\s*\n(\d+)", content)
            if width and height:
                self.calibration_size = (int(width.group(1)), int(height.group(1)))
        else:
            raise ValueError("Calibration must be CameraInfo, INI, NPZ or JSON")
        self.validate_calibration()

    def validate_calibration(self):
        if self.camera_matrix.shape != (3, 3) or not np.isfinite(self.camera_matrix).all() or \
                self.camera_matrix[0, 0] <= 0 or self.camera_matrix[1, 1] <= 0:
            raise ValueError("Invalid camera intrinsics; never estimate pose with identity fallback")
        if not np.isfinite(self.dist_coeffs).all():
            raise ValueError("Invalid distortion coefficients")

    def depth_info_callback(self, msg):
        self.depth_info = msg

    def depth_callback(self, msg):
        age = utime.stamp_age(self.get_clock(), msg.header.stamp)
        if not msg.header.frame_id or not -0.05 <= age <= self.settings['max_frame_age']:
            return
        try:
            image = decode_depth(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        normalized = self.bridge.cv2_to_imgmsg(image, encoding='32FC1')
        normalized.header = msg.header
        self.depth_frame = DepthFrame(image, msg.header, self.depth_info, normalized)
        self.depth_frames.append(self.depth_frame)
        if self.depth_pub.get_subscription_count():
            self.depth_pub.publish(normalized)
        for plugin in self.plugins:
            plugin.on_depth(self.depth_frame)

    def camera_info_callback(self, msg):
        matrix = np.asarray(msg.k, np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.isfinite(matrix).all():
            return
        if not self.settings["depth_camera_info_topic"]:
            self.depth_info = msg
        self.camera_info = msg
        self.camera_matrix = matrix
        self.dist_coeffs = np.asarray(msg.d, np.float64)
        self.calibration_size = (msg.width, msg.height) if msg.width and msg.height else None

    def _acquire_frame(self):
        image, header, sequence = self.source.grab_with_header()
        if image is None or sequence == self.sequence:
            return None
        if header is None:
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.settings['camera_frame'])
        elif abs(utime.stamp_age(self.get_clock(), header.stamp)) > self.settings['max_frame_age']:
            return None
        if not header.frame_id:
            header.frame_id = self.settings['camera_frame']
        if self.settings['is_stereo_image']:
            image = image[:, :image.shape[1] // 2]
        depth = self._matching_depth(header, image.shape[:2])
        wait = self.settings['depth_max_skew'] + utime.seconds_from_rate(self.settings['rate'])
        if self.settings['depth_topic'] and depth is None and utime.stamp_age(self.get_clock(), header.stamp) < wait:
            return None  # Keep this sequence available until its depth callback arrives.
        self.sequence = sequence
        self.frame_dimensions = (image.shape[1], image.shape[0])
        return Frame(image, header, self._scaled_intrinsics(image.shape[:2]), depth=depth)

    def _matching_depth(self, header, shape):
        return next((sample for sample in reversed(self.depth_frames)
                     if matches(sample, header, shape, self.settings['depth_max_skew'])), None)

    def _scaled_intrinsics(self, shape):
        matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
        if matrix is not None and self.calibration_size:
            height, width = shape
            calibration_width, calibration_height = self.calibration_size
            matrix[0, :] *= width / calibration_width
            matrix[1, :] *= height / calibration_height
        return matrix

    def _publish_raw(self, context):
        if self.raw_pub is not None and self.raw_pub.get_subscription_count():
            message = self.bridge.cv2_to_imgmsg(context.image, encoding='bgr8')
            message.header = context.header
            self.raw_pub.publish(message)

    def _preview_due(self, now):
        return (self.settings['publish_image'] and self.image_pub.get_subscription_count() > 0
                and utime.elapsed(self.last_preview, now) >= utime.seconds_from_rate(self.settings['preview_rate']))

    def _process_plugins(self, context):
        for plugin in self.plugins:
            if all(key in context.products for key in plugin.requires):
                plugin.process(context)

    def _publish_preview(self, context, publish, now):
        if context.preview is None:
            return
        if publish:
            preview = context.preview
            height, width = preview.shape[:2]
            target_width = self.settings['preview_width']
            if 0 < target_width < width:
                preview = cv2.resize(preview, (target_width, max(1, int(height * target_width / width))),
                                     interpolation=cv2.INTER_AREA)
            message = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
            message.header = context.header
            self.image_pub.publish(message)
            self.last_preview = now
        if self.settings['visualize']:
            cv2.imshow(self.get_fully_qualified_name(), context.preview)
            cv2.waitKey(1)

    def process(self):
        context = self._acquire_frame()
        if context is None:
            return
        self._publish_raw(context)
        start = utime.monotonic_now()
        publish = self._preview_due(start)
        if publish or self.settings['visualize']:
            context.preview = context.image.copy()
        self._process_plugins(context)
        self._publish_preview(context, publish, start)
        self.last_frame = utime.monotonic_now()
        self.last_inference_ms = utime.elapsed(start, self.last_frame) * 1000
        self.processed += 1

    def publish_diagnostics(self):
        status = DiagnosticStatus(name=self.get_fully_qualified_name(), hardware_id=self.settings["camera_frame"])
        age = utime.elapsed(self.last_frame) if self.last_frame else float("inf")
        missing_calibration = any("aruco" in plugin.provides for plugin in self.plugins) and self.camera_matrix is None
        status.level = DiagnosticStatus.WARN if age > self.settings["max_frame_age"] or missing_calibration else DiagnosticStatus.OK
        status.message = "Missing/stale images or calibration" if status.level else "Processing"
        status.values = [KeyValue(key="frames_processed", value=str(self.processed)),
                         KeyValue(key="last_processing_ms", value=f"{self.last_inference_ms:.3f}"),
                         KeyValue(key="frame_age_s", value=str(age))]
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_pub.publish(message)

    def destroy_node(self):
        for plugin in reversed(self.plugins):
            plugin.close()
        if self.source is not None:
            self.source.release()
        if self.settings["visualize"]:
            cv2.destroyAllWindows()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MiraVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
