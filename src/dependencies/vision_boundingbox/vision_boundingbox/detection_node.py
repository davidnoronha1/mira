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

Parameters:
  detections_topic  (string, default "/vision/detections") - topic for Detection2DArray output
  image_topic       (string, default "/vision/image")      - topic for annotated image output

Publishes:
  <detections_topic>  (vision_msgs/Detection2DArray)
  <image_topic>       (sensor_msgs/Image)  -- if publish_image=true
"""

import math
from pathlib import Path
from typing import Optional, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, Imu
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
    Pose2D,
)
from utils.image_source import ImageSource, build_image_source

try:
    from ultralytics import YOLO
except ImportError as e:
    raise ImportError("ultralytics is required: pip install ultralytics") from e


def is_reflection(box: np.ndarray, threshold: float) -> bool:
    return False


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
        self._integrated_yaw: float = 0.0   # radians accumulated since last detection
        self._integrated_pitch: float = 0.0  # radians accumulated since last detection
        self._imu_angular_velocity = None   # latest (vx, vy, vz) from /master/imu
        self._imu_lock = threading.Lock()
        self._estimation_warned: bool = False  # tracks whether the >1s warning has been issued

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

        # Run at 30Hz
        self._timer = self.create_timer(1.0 / 30.0, self._process)
        self.get_logger().info("vision_boundingbox_node (Ultralytics) ready.")

    def _declare_params(self):
        self.declare_parameter("image_source", "rtsp://192.168.2.6:2002/image_rtsp")
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
        self.declare_parameter("is_stereo_image", False)
        self.declare_parameter("enable_bb_estimation", True)
        self.declare_parameter("imu_topic", "/master/imu")  # sensor_msgs/Imu topic for BB estimation
        self.declare_parameter("hfov_deg", 90.0)  # Camera horizontal field of view (degrees)
        self.declare_parameter("vfov_deg", 60.0)  # Camera vertical field of view (degrees)
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

        return model_name  # Relative fallback

    def _build_source(self) -> ImageSource:
        uri: str = self.get_parameter("image_source").value
        return build_image_source(uri, self)

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

        if self.get_parameter("is_stereo_image").value:
            frame = frame[:, : frame.shape[1] // 2]

        stamp = now.to_msg()

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
            self._last_process_time = now
            return

        result = results[0]

        # Use xyxyn — Ultralytics normalises to [0,1] and handles letterbox
        # removal internally, so coordinates are always correct regardless of
        # model export format or input resolution.
        det_array = self._build_detection_msg(result, stamp)
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

    def _build_detection_msg(self, result, stamp) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"

        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return msg

        # xyxyn: [x1, y1, x2, y2] normalised to [0,1] by Ultralytics,
        # letterbox padding already removed — works for any model format.
        xyxyn = boxes.xyxyn.cpu().numpy()
        confs  = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy().astype(int)

        for i in range(len(boxes)):
            x1n, y1n, x2n, y2n = xyxyn[i]

            d = Detection2D()
            d.header = msg.header

            bb = BoundingBox2D()
            bb.center.position.x = float((x1n + x2n) / 2.0)
            bb.center.position.y = float((y1n + y2n) / 2.0)
            bb.center.theta = 0.0
            bb.size_x = float(x2n - x1n)
            bb.size_y = float(y2n - y1n)
            d.bbox = bb

            class_name = result.names.get(cls_ids[i], str(cls_ids[i]))
            d.id = class_name

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(confs[i])
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
