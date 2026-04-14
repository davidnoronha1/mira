#!/usr/bin/env python3
"""
vision_boundingbox_node.py
ROS2 node for YOLOv8/YOLOv11 object detection using ONNX Runtime.

Image sources (image_source parameter):
  rtsp://host:port/path         - RTSP stream via OpenCV
  rtsps://host:port/path        - RTSP-over-TLS stream via OpenCV
  file:///abs/path/to/video.mp4 - Local video file via OpenCV
  ros2://topic/name             - ROS2 sensor_msgs/Image topic
  camera://0                    - Default webcam (index 0)

Parameters:
  detections_topic  (string, default "/vision/detections") - topic for Detection2DArray output
  image_topic       (string, default "/vision/image")      - topic for annotated image output

Publishes:
  <detections_topic>  (vision_msgs/Detection2DArray)
  <image_topic>       (sensor_msgs/Image)  -- if publish_image=true
"""

import threading
from pathlib import Path
from typing import Optional, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, Imu
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)

from vision_boundingbox.yolo_detector import YOLODetector, Detection


# ---------------------------------------------------------------------------
# Image source abstraction
# ---------------------------------------------------------------------------

class ImageSource:
    def grab(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def release(self):
        pass


class OpenCVSource(ImageSource):
    def __init__(self, uri: str, node_logger):
        node_logger.info(f"Using video capture: {uri}")
        self._cap = cv2.VideoCapture(uri)

        if not self._cap.isOpened():
            node_logger.error(f"Cannot open video source: {uri}")
        else:
            node_logger.info(f"Opened video source: {uri}")

    def grab(self) -> Optional[np.ndarray]:
        ret, frame = self._cap.read()
        return frame if ret else None

    def release(self):
        self._cap.release()


class ROS2TopicSource(ImageSource):
    def __init__(self, topic: str, node: "VisionBoundingBoxNode"):
        self._bridge = CvBridge()
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._sub = node.create_subscription(
            Image,
            topic,
            self._callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        node.get_logger().info(f"Subscribed to ROS2 image topic: {topic}")

    def _callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._frame = frame
        except Exception:
            pass

    def grab(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


# ---------------------------------------------------------------------------
# Main ROS2 Node
# ---------------------------------------------------------------------------

class VisionBoundingBoxNode(Node):

    def __init__(self):
        super().__init__("vision_boundingbox_node")
        self._declare_params()
        self._bridge = CvBridge()

        # Load model using ONNX Runtime
        model_path = self._resolve_model()
        class_names_path = self._resolve_class_names()
        device_str = self.get_parameter("device").value

        self.get_logger().info(f"Loading ONNX model: {model_path}")
        self._detector = YOLODetector(
            model_path=model_path,
            class_names_path=class_names_path,
            input_size=(
                self.get_parameter("input_width").value,
                self.get_parameter("input_height").value,
            ),
            conf_threshold=self.get_parameter("conf_threshold").value,
            nms_threshold=self.get_parameter("nms_threshold").value,
            device=device_str,
        )

        # Image source
        self._source = self._build_source()

        # Publishers
        det_topic = self.get_parameter("detections_topic").value
        img_topic = self.get_parameter("image_topic").value
        self._det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self._img_pub = (
            self.create_publisher(Image, img_topic, 10)
            if self.get_parameter("publish_image").value
            else None
        )

        # BB estimation state
        self._last_detections: List[Detection2D] = []
        self._last_detection_time: Optional[rclpy.time.Time] = None
        self._last_process_time: Optional[rclpy.time.Time] = None
        self._integrated_yaw: float = 0.0
        self._integrated_pitch: float = 0.0
        self._imu_angular_velocity = None
        self._imu_lock = threading.Lock()
        self._estimation_warned: bool = False

        # IMU subscription for BB estimation
        if self.get_parameter("enable_bb_estimation").value:
            imu_topic = self.get_parameter("imu_topic").value
            self._imu_sub = self.create_subscription(
                Imu,
                imu_topic,
                self._imu_callback,
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self.get_logger().info(f"BB estimation enabled; subscribed to {imu_topic}")

        # Run at 30 Hz
        self._timer = self.create_timer(1.0 / 30.0, self._process)
        self.get_logger().info("vision_boundingbox_node (ONNX Runtime) ready.")

    def _declare_params(self):
        self.declare_parameter("image_source", "rtsp://192.168.2.6:2001/image_rtsp")
        self.declare_parameter("model_name", "docking.onnx")
        self.declare_parameter("class_names", "class_names.txt")
        self.declare_parameter("device", "CPU")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.4)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("visualize", False)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("input_width", 640)
        self.declare_parameter("reject_reflections", True)
        self.declare_parameter("reject_threshold", 0.5)
        self.declare_parameter("enable_bb_estimation", True)
        self.declare_parameter("imu_topic", "/master/imu")
        self.declare_parameter("hfov_deg", 90.0)
        self.declare_parameter("vfov_deg", 60.0)
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("image_topic", "/vision/image")

    def _resolve_model(self) -> str:
        model_name = self.get_parameter("model_name").value
        p = Path(model_name)

        if p.is_absolute():
            return str(p)

        try:
            pkg_share = Path(get_package_share_directory("vision_boundingbox"))
            model_path = pkg_share / "models" / model_name
            if model_path.exists():
                return str(model_path)
        except Exception:
            pass

        return model_name

    def _resolve_class_names(self) -> Optional[str]:
        class_names_file = self.get_parameter("class_names").value
        p = Path(class_names_file)

        if p.is_absolute() and p.exists():
            return str(p)

        try:
            pkg_share = Path(get_package_share_directory("vision_boundingbox"))
            names_path = pkg_share / "models" / class_names_file
            if names_path.exists():
                return str(names_path)
        except Exception:
            pass

        return None

    def _build_source(self) -> ImageSource:
        uri: str = self.get_parameter("image_source").value

        if uri.startswith("camera://"):
            idx = uri[len("camera://"):]
            return OpenCVSource(idx if idx.isdigit() else "0", self.get_logger())

        if uri.startswith("ros2://"):
            topic = uri[len("ros2://"):]
            if not topic.startswith("/"): topic = "/" + topic
            return ROS2TopicSource(topic, self)

        if any(uri.startswith(s) for s in ["file://", "rtsp://", "rtsps://", "https://", "http://"]):
            return OpenCVSource(uri, self.get_logger())

        return OpenCVSource(uri, self.get_logger())

    def _imu_callback(self, msg: Imu):
        with self._imu_lock:
            self._imu_angular_velocity = (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )

    def _process(self):
        now = self.get_clock().now()
        frame = self._source.grab()
        if frame is None:
            self._last_process_time = now
            return

        stamp = now.to_msg()
        h, w = frame.shape[:2]

        detections = self._detector.detect(frame)

        det_array = self._build_detection_msg(detections, stamp, w, h)
        self._det_pub.publish(det_array)

        if self._img_pub is not None or self.get_parameter("visualize").value:
            vis = self._detector.draw_detections(frame, detections)

            if self._img_pub is not None:
                img_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = "camera"
                self._img_pub.publish(img_msg)

            if self.get_parameter("visualize").value:
                cv2.imshow("vision_boundingbox", vis)
                cv2.waitKey(1)

        self._last_process_time = now

    def _build_detection_msg(
        self,
        detections: List[Detection],
        stamp,
        frame_w: int,
        frame_h: int,
    ) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"

        for det in detections:
            x, y, bw, bh = det.box  # pixel coords: top-left x/y, width, height

            # Normalize to [0, 1]
            cx = (x + bw / 2.0) / frame_w
            cy = (y + bh / 2.0) / frame_h
            sx = bw / frame_w
            sy = bh / frame_h

            d = Detection2D()
            d.header = msg.header

            bb = BoundingBox2D()
            bb.center.position.x = float(cx)
            bb.center.position.y = float(cy)
            bb.center.theta = 0.0
            bb.size_x = float(sx)
            bb.size_y = float(sy)
            d.bbox = bb

            d.id = det.class_name

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det.class_name
            hyp.hypothesis.score = float(det.confidence)
            d.results.append(hyp)

            msg.detections.append(d)

        return msg

    def destroy_node(self):
        self._source.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionBoundingBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()