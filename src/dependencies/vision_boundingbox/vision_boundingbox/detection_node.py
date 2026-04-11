#!/usr/bin/env python3
"""
vision_boundingbox_node.py
ROS2 node for YOLOv8/YOLOv11 object detection using Ultralytics.

Image sources (image_source parameter):
  rtsp://host:port/path         - RTSP stream via OpenCV
  rtsps://host:port/path        - RTSP-over-TLS stream via OpenCV
  file:///abs/path/to/video.mp4 - Local video file via OpenCV
  ros2://topic/name             - ROS2 sensor_msgs/Image topic
  camera://0                    - Default webcam (index 0)

Publishes:
  ~/detections   (vision_msgs/Detection2DArray)
  ~/image        (sensor_msgs/Image)  -- if publish_image=true
"""

import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
    Pose2D,
    Point2D,
)

try:
    from ultralytics import YOLO
except ImportError as e:
    raise ImportError("ultralytics is required: pip install ultralytics") from e


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def is_reflection(box: np.ndarray, threshold: float) -> bool:
    """
    Heuristic placeholder for reflection rejection.
    box: [x1, y1, x2, y2, conf, cls_id]
    """
    return False


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
        if uri.startswith("file://"):
            path = uri[len("file://"):]
            self._cap = cv2.VideoCapture(path)
        else:
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

        # Load model using Ultralytics
        model_path = self._resolve_model()
        self.get_logger().info(f"Loading Ultralytics model: {model_path}")
        
        # device: 'cpu', 0 (for cuda:0), or 'mps'
        device_str = self.get_parameter("device").value.lower()
        if device_str == "gpu":
            device_str = "0"

        self._model = YOLO(model_path)
        # self._model.to(device_str)

        # Image source
        self._source = self._build_source()

        # Publishers
        self._det_pub = self.create_publisher(Detection2DArray, "/vision/detections", 10)
        self._img_pub = (
            self.create_publisher(Image, "/vision/image", 10)
            if self.get_parameter("publish_image").value
            else None
        )

        # Run at 30Hz
        self._timer = self.create_timer(1.0 / 30.0, self._process)
        self.get_logger().info("vision_boundingbox_node (Ultralytics) ready.")

    def _declare_params(self):
        self.declare_parameter("image_source", "rtsp://192.168.2.6:2001/image_rtsp")
        self.declare_parameter("model_name", "docking.pt")  # Ultralytics prefers .pt or .onnx
        self.declare_parameter("device", "CPU")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.4)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("visualize", False)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("input_width", 640)
        self.declare_parameter("reject_reflections", True)
        self.declare_parameter("reject_threshold", 0.5)

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

        return model_name  # Relative fallback

    def _build_source(self) -> ImageSource:
        uri: str = self.get_parameter("image_source").value

        if uri.startswith("camera://"):
            idx = uri[len("camera://"):]
            return OpenCVSource(idx if idx.isdigit() else "0", self.get_logger())

        if uri.startswith("ros2://"):
            topic = uri[len("ros2://"):]
            if not topic.startswith("/"): topic = "/" + topic
            return ROS2TopicSource(topic, self)

        if any(uri.startswith(s) for s in ["file://", "rtsp://", "rtsps://"]):
            return OpenCVSource(uri, self.get_logger())

        return OpenCVSource(uri, self.get_logger())

    def _process(self):
        frame = self._source.grab()
        if frame is None:
            return

        stamp = self.get_clock().now().to_msg()

        # Inference
        # Ultralytics handles resizing, NMS, and scaling back to original size automatically
        results = self._model.predict(
            source=frame,
            conf=self.get_parameter("conf_threshold").value,
            iou=self.get_parameter("nms_threshold").value,
            imgsz=(self.get_parameter("input_height").value, self.get_parameter("input_width").value),
            device=self._model.device,
            verbose=False
        )

        if not results:
            return

        result = results[0]
        # boxes format: [x1, y1, x2, y2, conf, cls]
        detections = result.boxes.data.cpu().numpy()

        # Optional reflection rejection
        if self.get_parameter("reject_reflections").value and len(detections) > 0:
            rt = self.get_parameter("reject_threshold").value
            detections = detections[~np.array([is_reflection(d, rt) for d in detections])]

        # Publish detections
        det_array = self._build_detection_msg(detections, result.names, stamp)
        self._det_pub.publish(det_array)

        # Publish / visualize image
        if self._img_pub is not None or self.get_parameter("visualize").value:
            # result.plot() returns BGR image with boxes drawn
            vis = result.plot()
            
            if self._img_pub is not None:
                img_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = "camera"
                self._img_pub.publish(img_msg)

            if self.get_parameter("visualize").value:
                cv2.imshow("vision_boundingbox", vis)
                cv2.waitKey(1)

    def _build_detection_msg(
        self, detections: np.ndarray, names: dict, stamp
    ) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"

        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            
            d = Detection2D()
            d.header = msg.header

            bb = BoundingBox2D()
            bb.center.position.x = float((x1 + x2) / 2.0)
            bb.center.position.y = float((y1 + y2) / 2.0)
            bb.center.theta = 0.0
            bb.size_x = float(x2 - x1)
            bb.size_y = float(y2 - y1)
            d.bbox = bb

            cls_int = int(cls_id)
            class_name = names.get(cls_int, str(cls_int))

            # Populate id so BT nodes can match by object name
            d.id = class_name

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(conf)
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
