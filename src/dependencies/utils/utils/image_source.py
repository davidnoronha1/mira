"""Shared ImageSource abstraction for file://, rtsp://, ros2://, and camera:// URIs."""

import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

_STATIC_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif", ".webp"}


class ImageSource:
    """Abstract base for all image sources."""

    def grab(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def release(self):
        pass

    def is_open(self) -> bool:
        return True


class OpenCVSource(ImageSource):
    """OpenCV VideoCapture — handles rtsp://, rtsps://, and file:// video."""

    def __init__(self, uri: str, logger=None):
        path = uri[len("file://"):] if uri.startswith("file://") else uri
        self._cap = cv2.VideoCapture(path)
        if not self._cap.isOpened():
            if logger:
                logger.error(f"Cannot open video source: {uri}")
        elif logger:
            logger.info(f"Opened video source: {uri}")

    def grab(self) -> Optional[np.ndarray]:
        ret, frame = self._cap.read()
        return frame if ret else None

    def is_open(self) -> bool:
        return self._cap.isOpened()

    def release(self):
        self._cap.release()


class StaticImageSource(ImageSource):
    """Returns the same frame on every grab() — for still images via file://."""

    def __init__(self, path: str, logger=None):
        self._frame = cv2.imread(path)
        if self._frame is None:
            if logger:
                logger.error(f"Cannot read static image: {path}")
        elif logger:
            logger.info(f"Loaded static image: {path}")

    def grab(self) -> Optional[np.ndarray]:
        return self._frame.copy() if self._frame is not None else None


class ROS2TopicSource(ImageSource):
    """Subscribes to a ROS2 sensor_msgs/Image topic (ros2://topic/name)."""

    def __init__(self, topic: str, node):
        from cv_bridge import CvBridge  # noqa: PLC0415
        from sensor_msgs.msg import Image  # noqa: PLC0415
        from rclpy.qos import QoSPresetProfiles  # noqa: PLC0415

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

    def _callback(self, msg) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._frame = frame
        except Exception:
            pass

    def grab(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


def build_image_source(uri: str, node=None) -> ImageSource:
    """
    Factory: return the correct ImageSource for *uri*.

    Supported schemes::

        rtsp://host:port/path        RTSP stream via OpenCV
        rtsps://host:port/path       RTSP over TLS via OpenCV
        file:///absolute/path        video file or still image via OpenCV
        ros2://topic/name            ROS2 sensor_msgs/Image subscription
        camera://0                   webcam by integer index via OpenCV

    *node* must be a ``rclpy.Node`` when using ``ros2://``; ignored otherwise.
    """
    logger = node.get_logger() if node is not None else None

    if uri.startswith("camera://"):
        idx = uri[len("camera://"):]
        return OpenCVSource(idx if idx.isdigit() else "0", logger)

    if uri.startswith("ros2://"):
        topic = uri[len("ros2://"):]
        if not topic.startswith("/"):
            topic = "/" + topic
        if node is None:
            raise ValueError("ros2:// image source requires a ROS2 node instance")
        return ROS2TopicSource(topic, node)

    if uri.startswith("file://"):
        path = uri[len("file://"):]
        if Path(path).suffix.lower() in _STATIC_IMAGE_SUFFIXES:
            return StaticImageSource(path, logger)
        return OpenCVSource(uri, logger)

    if any(uri.startswith(s) for s in ("rtsp://", "rtsps://")):
        return OpenCVSource(uri, logger)

    # Bare path fallback — no explicit scheme
    if Path(uri).suffix.lower() in _STATIC_IMAGE_SUFFIXES:
        return StaticImageSource(uri, logger)
    return OpenCVSource(uri, logger)
