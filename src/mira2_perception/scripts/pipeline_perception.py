#!/usr/bin/env python3

''' 
Changes to be made

1) Change the Camera topic
2) Tune HSV

'''


import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArUcoDetector:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(16)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.detected_markers = []
        self.marker_positions = {}
        self.confirmation_buffer = {}
        self._last_corners = []
        self._last_ids = None

    def _preprocess(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(gray)

    def detect(self, frame):
        try:
            gray = self._preprocess(frame)
            corners, ids, _ = self.detector.detectMarkers(gray)
        except Exception as e:
            self._last_corners = []
            self._last_ids = None
            return []

        self._last_corners = corners if corners is not None else []
        self._last_ids = ids
        new_detections = []

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)
                corner = corners[i][0]
                cx = float(np.mean(corner[:, 0]))
                cy = float(np.mean(corner[:, 1]))

                if self._is_new_marker(marker_id, cx, cy):
                    self.confirmation_buffer.setdefault(marker_id, 0)
                    self.confirmation_buffer[marker_id] += 1

                    if self.confirmation_buffer[marker_id] >= 3:
                        if marker_id not in self.detected_markers:
                            self.detected_markers.append(marker_id)
                            self.marker_positions[marker_id] = (cx, cy)
                            new_detections.append(marker_id)
                        del self.confirmation_buffer[marker_id]

        return new_detections

    def _is_new_marker(self, marker_id, x, y, min_distance=200):
        if marker_id not in self.marker_positions:
            return True
        px, py = self.marker_positions[marker_id]
        return np.hypot(x - px, y - py) > min_distance

    def visualize(self, frame):
        if self._last_ids is not None and len(self._last_corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, self._last_corners, self._last_ids)
        return frame

    def get_marker_list(self):
        return self.detected_markers


class PipelineDetectorNode(Node):

    def __init__(self):
        super().__init__("pipeline_detector_node")
        self.get_logger().info("Pipeline Detector Node Started")

        self.bridge = CvBridge()

        self.hsv_lower = np.array([15, 30, 30])
        self.hsv_upper = np.array([90, 255, 255])

        self.aruco_detector = ArUcoDetector()

        # Publishers
        self.offset_pub      = self.create_publisher(Point,   '/pipeline/offset',      10)
        self.angle_pub       = self.create_publisher(Float32, '/pipeline/angle',        10)
        self.markers_pub     = self.create_publisher(String,  '/pipeline/markers',      10)
        self.debug_image_pub = self.create_publisher(Image,   '/pipeline/debug_image',  10)

        # Subscriber
        self.frame_sub = self.create_subscription(
            Image, '/bluerov2/camera_bottom/image_color', self.frame_callback, 10
        )

    def frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if frame is None:
            self.get_logger().warn("Frame conversion failed")
            return
        self.process(frame)

    def process(self, frame):
        detected, centroid, norm_x, norm_y, angle, mask = self.detect_pipeline(frame)

        # ArUco detection
        new_markers = self.aruco_detector.detect(frame)
        if new_markers:
            all_markers = self.aruco_detector.get_marker_list()
            markers_msg = String()
            markers_msg.data = ",".join(map(str, all_markers))
            self.markers_pub.publish(markers_msg)
            self.get_logger().info(f"New markers detected: {new_markers}")

        # Publish pipeline data
        if detected:
            offset_msg = Point()
            offset_msg.x = norm_x
            offset_msg.y = norm_y
            offset_msg.z = 0.0
            self.offset_pub.publish(offset_msg)

            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.angle_pub.publish(angle_msg)

        # Visualization
        vis_frame = self.draw_visualization(frame, detected, centroid, norm_x, norm_y, angle)
        vis_frame = self.aruco_detector.visualize(vis_frame)

        markers = self.aruco_detector.get_marker_list()
        text = f"Markers: {markers}" if markers else "No markers yet"
        cv2.putText(vis_frame, text,
                    (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Pipeline Detection", vis_frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        debug_msg = self.bridge.cv2_to_imgmsg(vis_frame, "bgr8")
        self.debug_image_pub.publish(debug_msg)

    def detect_pipeline(self, frame):
        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None, None, None, None, mask

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 1000:
            return None, None, None, None, None, mask

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None, None, None, None, mask

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        norm_x = (cx - width  / 2) / (width  / 2)
        norm_y = (cy - height / 2) / (height / 2)

        [vx, vy, _, _] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
        raw = np.degrees(np.arctan2(float(vy[0]), float(vx[0])))
        angle = 90 - abs(raw)
        if raw < 0:
            angle = -angle

        return True, (cx, cy), norm_x, norm_y, angle, mask

    def draw_visualization(self, frame, detected, centroid, norm_x, norm_y, angle):
        vis = frame.copy()
        height, width = frame.shape[:2]

        if not detected:
            cv2.putText(vis, "NO PIPELINE DETECTED", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            return vis

        cx, cy = centroid
        cv2.circle(vis, (cx, cy), 15, (0, 0, 255), -1)
        cv2.arrowedLine(vis, (width // 2, height // 2), (cx, cy), (0, 255, 0), 3)
        cv2.putText(vis,
                    f"offset: ({norm_x:.2f}, {norm_y:.2f})  angle: {angle:.1f}deg",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        return vis


def main(args=None):
    rclpy.init(args=args)
    node = PipelineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()