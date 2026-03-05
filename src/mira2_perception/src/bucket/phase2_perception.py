import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from collections import deque

from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PhaseTwoPerceptionNode(Node):

    def __init__(self):
        super().__init__("buckets_phase_two_perception_node")

        self.get_logger().info("Phase Two Perception Node Started")

        self.bridge = CvBridge()
        self.frame = None
        self.ellipse = None

        self.MIN_BUCKET_AREA = 3000
        self.SMOOTH_N = 5

        self.cx_history = deque(maxlen=self.SMOOTH_N)
        self.cy_history = deque(maxlen=self.SMOOTH_N)

        # HSV Ranges
        self.LOWER_BLUE = np.array([105, 50, 50])
        self.UPPER_BLUE = np.array([125, 255, 255])

        self.LOWER_ORANGE_1 = np.array([0, 50, 50])
        self.UPPER_ORANGE_1 = np.array([40, 255, 255])

        self.LOWER_ORANGE_2 = np.array([165, 50, 50])
        self.UPPER_ORANGE_2 = np.array([180, 255, 255])

        # Morphology kernels
        self.kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))

        # Publishers
        self.offset_pub = self.create_publisher(Point, '/bucket/p2offset', 10)
        self.color_pub = self.create_publisher(String, '/bucket/p2color', 10)

        # Subscriber
        self.frame_sub = self.create_subscription(Image, '/camera/image_raw', self.frame_callback, 10)


    def frame_callback(self, msg):

        self.get_logger().info("Frame received")

        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.frame is None:
            self.get_logger().warn("Frame conversion failed")
            return

        self.buckets_main()


    def detect_buckets(self):

        h, w = self.frame.shape[:2]
        frame_cx, frame_cy = w // 2, h // 2

        frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # BLUE MASK
        blue_mask = cv2.inRange(frame_hsv, self.LOWER_BLUE, self.UPPER_BLUE)

        # ORANGE MASK (two ranges)
        orange_mask1 = cv2.inRange(frame_hsv, self.LOWER_ORANGE_1, self.UPPER_ORANGE_1)
        orange_mask2 = cv2.inRange(frame_hsv, self.LOWER_ORANGE_2, self.UPPER_ORANGE_2)

        orange_mask = cv2.bitwise_or(orange_mask1, orange_mask2)

        # Morphology
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, self.kernel_small)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel_large)

        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel_small)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, self.kernel_large)

        # Detect contours for both colors
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        detected_color = "none"
        valid_contours = []

        # Check blue first
        if blue_contours:
            valid_contours = [c for c in blue_contours if cv2.contourArea(c) > self.MIN_BUCKET_AREA]

            if valid_contours:
                detected_color = "blue"

        # If no blue, check orange
        if not valid_contours and orange_contours:
            valid_contours = [c for c in orange_contours if cv2.contourArea(c) > self.MIN_BUCKET_AREA]

            if valid_contours:
                detected_color = "orange"

        if valid_contours:

            largest = max(valid_contours, key=cv2.contourArea)

            if len(largest) >= 5:

                candidate = cv2.fitEllipse(largest)

                if self.is_valid_ellipse(candidate, w, h):

                    self.ellipse = candidate

                    (cx, cy), axes, angle = self.ellipse

                    self.cx_history.append(int(cx))
                    self.cy_history.append(int(cy))

                    detected = True

                    self.get_logger().info(f"{detected_color} bucket detected")


        if self.cx_history:

            smooth_cx = int(np.mean(self.cx_history))
            smooth_cy = int(np.mean(self.cy_history))

            offset_x = smooth_cx - frame_cx
            offset_y = smooth_cy - frame_cy

            norm_x = offset_x / (w / 2)
            norm_y = offset_y / (h / 2)

            return norm_x, norm_y, detected, detected_color

        return None, None, False, "none"


    def is_valid_ellipse(self, ellipse, w, h):

        (cx, cy), (ma, Mi), angle = ellipse

        if not (0 < cx < w and 0 < cy < h):
            return False

        if ma < 10 or Mi < 10:
            return False

        ratio = max(ma, Mi) / (min(ma, Mi) + 1e-5)

        if ratio > 4.0:
            return False

        return True


    def buckets_main(self):

        norm_x, norm_y, detected, detected_color = self.detect_buckets()

        if norm_x is not None:

            offset_msg = Point()
            offset_msg.x = float(norm_x)
            offset_msg.y = float(norm_y)
            offset_msg.z = 1.0 if detected else 0.0

            self.offset_pub.publish(offset_msg)

            color_msg = String()
            color_msg.data = detected_color
            self.color_pub.publish(color_msg)

            self.get_logger().info(f"Published -> Offset: ({norm_x:.2f},{norm_y:.2f}) Color: {detected_color}")

        else:
            self.get_logger().warn("No bucket detected")


def main(args=None):

    rclpy.init(args=args)

    node = PhaseTwoPerceptionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()