#!/usr/bin/env python3
"""One frame pipeline per camera: YOLO, ArUco, docking board and pipeline."""

from collections import deque
import json
import math
from pathlib import Path
import re
import time

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
from mira2_vision.depth import DepthFrame, decode_depth, matches
from mira2_vision.plugin import Frame, load_plugin, order_plugins


class MiraVisionNode(Node):
    def __init__(self):
        super().__init__("perception")
        defaults = {
            "image_source": "ros2:///mira/sensors/camera/front/image_raw",
            "camera_info_topic": "/mira/sensors/camera/front/camera_info",
            "raw_image_topic": "/mira/sensors/camera/front/image_raw",
            "simulation": False,
            "depth_camera_info_topic": "",
            "depth_topic": "", "depth_max_skew": 0.1, "depth_min": 0.1, "depth_max": 20.0,
            "target_class": "blue_bucket",
            "camera_frame": "camera_front_optical", "calibration_file": "",
            "detectors": Setting(["yolo"], string_list, dynamic_typing=True),
            "consumers": Setting(["objects"], string_list, dynamic_typing=True),
            "model_name": "best.onnx", "device": "CPU", "conf_threshold": 0.5,
            "nms_threshold": 0.4, "input_height": 640, "input_width": 640,
            "rate": Setting(30.0, positive), "yolo_rate": Setting(15.0, positive), "aruco_rate": Setting(30.0, positive),
            "max_frame_age": Setting(0.5, positive), "publish_image": True, "visualize": False,
            "publish_tf": True,
            "preview_rate": Setting(5.0, positive), "preview_width": 640,
            "is_stereo_image": False, "aruco_dictionary": "DICT_ARUCO_ORIGINAL",
            "marker_size": Setting(0.15, positive), "clahe_clip_limit": 2.5,
            "clahe_grid_size": 8,
            "board_ids": [28, 7, 19, 96],
            "board_offsets": [-0.29, -0.49, 0.0, 0.29, -0.49, 0.0,
                              -0.29, 0.49, 0.0, 0.29, 0.49, 0.0],
            "pose_filter_alpha": 0.6, "pose_filter_max_jump": 0.6,
            "pose_filter_reset_after": 5,
            "hsv_lower": [15, 30, 30], "hsv_upper": [90, 255, 255],
            "pipeline_min_area": 1000.0, "pipeline_kernel_size": 5,
        }
        self.settings = declare_config(self, defaults)
        if not 0 < self.settings["pose_filter_alpha"] <= 1:
            raise ValueError("pose_filter_alpha must be in (0, 1]")
        if len(self.settings["board_offsets"]) != 3 * len(self.settings["board_ids"]):
            raise ValueError("board_offsets must have three coordinates per board ID")
        if self.settings["preview_width"] < 0:
            raise ValueError("preview_width must be nonnegative; 0 means native resolution")
        for name in ("clahe_grid_size", "pipeline_kernel_size", "input_width", "input_height"):
            if self.settings[name] <= 0:
                raise ValueError(f"{name} must be positive")
        if len(self.settings["hsv_lower"]) != 3 or len(self.settings["hsv_upper"]) != 3:
            raise ValueError("HSV thresholds must each have three components")
        if not self.settings['simulation'] and self.get_parameter('use_sim_time').value:
            raise ValueError('use_sim_time requires simulation=true')
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
        specifications = self.settings['detectors'] + self.settings['consumers']
        if len(specifications) != len(set(specifications)):
            raise ValueError('A vision plugin may only be attached once')
        self.plugins = order_plugins([load_plugin(spec, self) for spec in specifications])
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
        self.source = build_image_source(self.settings["image_source"], self)
        self.sequence = None
        self.last_frame = 0.0
        self.last_preview = 0.0
        self.processed = 0
        self.last_inference_ms = 0.0
        self.create_timer(1 / self.settings["rate"], self.process)
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
        age = (self.get_clock().now().nanoseconds * 1e-9 -
               msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9)
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

    def process(self):
        frame, header, sequence = self.source.grab_with_header()
        if frame is None or sequence == self.sequence:
            return
        now = self.get_clock().now()
        if header is not None:
            stamp_ns = header.stamp.sec * 1000000000 + header.stamp.nanosec
            age = (now.nanoseconds - stamp_ns) / 1e9
            if age > self.settings["max_frame_age"] or age < -self.settings["max_frame_age"]:
                return
        else:
            header = Header(stamp=now.to_msg(), frame_id=self.settings["camera_frame"])
        if not header.frame_id:
            header.frame_id = self.settings["camera_frame"]
        if self.settings["is_stereo_image"]:
            frame = frame[:, :frame.shape[1] // 2]
        matching_depth = next((sample for sample in reversed(self.depth_frames)
                               if matches(sample, header, frame.shape[:2], self.settings['depth_max_skew'])), None)
        # Let the paired depth callback arrive before consuming this RGB sequence.
        age = (now.nanoseconds - header.stamp.sec * 1000000000 - header.stamp.nanosec) / 1e9
        if (self.settings['depth_topic'] and matching_depth is None and
                age < self.settings['depth_max_skew'] + 1 / self.settings['rate']):
            return
        self.sequence = sequence
        height, width = frame.shape[:2]
        self.frame_dimensions = (width, height)
        if self.raw_pub is not None and self.raw_pub.get_subscription_count() > 0:
            raw = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw.header = header
            self.raw_pub.publish(raw)
        matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
        if matrix is not None and self.calibration_size:
            cw, ch = self.calibration_size
            matrix[0, :] *= width / cw
            matrix[1, :] *= height / ch
        start = time.monotonic()
        want_image = self.settings["publish_image"] and self.image_pub.get_subscription_count() > 0 and \
            start - self.last_preview >= 1 / self.settings["preview_rate"]
        visualization = frame.copy() if want_image or self.settings["visualize"] else None
        context = Frame(frame, header, matrix, visualization)
        context.depth = matching_depth
        for plugin in self.plugins:
            if all(key in context.products for key in plugin.requires):
                plugin.process(context)
        visualization = context.preview
        if visualization is not None:
            if want_image:
                preview = visualization
                preview_width = self.settings["preview_width"]
                if 0 < preview_width < width:
                    preview = cv2.resize(preview, (preview_width, max(1, int(height * preview_width / width))),
                                         interpolation=cv2.INTER_AREA)
                image = self.bridge.cv2_to_imgmsg(preview, encoding="bgr8")
                image.header = header
                self.image_pub.publish(image)
                self.last_preview = start
            if self.settings["visualize"]:
                cv2.imshow(self.get_fully_qualified_name(), visualization)
                cv2.waitKey(1)
        self.last_frame = time.monotonic()
        self.last_inference_ms = (self.last_frame - start) * 1000
        self.processed += 1

    def publish_diagnostics(self):
        status = DiagnosticStatus(name=self.get_fully_qualified_name(), hardware_id=self.settings["camera_frame"])
        age = time.monotonic() - self.last_frame if self.last_frame else float("inf")
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
