from utils.parameters import Setting, declare_config, positive
import math
import cv2
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from mira2_vision.plugin import Plugin


class Pipeline(Plugin):
    requires = ()
    provides = ()

    def __init__(self, node):
        super().__init__(node)
        self.config = declare_config(node, {
            'hsv_lower': [15, 30, 30],
            'hsv_upper': [90, 255, 255],
            'min_area': Setting(1000.0, positive),
            'kernel_size': Setting(5, positive),
        }, namespace='pipeline')
        if len(self.config["hsv_lower"]) != 3 or len(self.config["hsv_upper"]) != 3:
            raise ValueError("pipeline HSV thresholds must each have three components")
        self.pipeline_pub = node.create_publisher(Point, 'pipeline/offset', 10)
        self.angle_pub = node.create_publisher(Float32, 'pipeline/angle', 10)

    def process(self, context):
        frame, visualization = context.image, context.preview
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.asarray(self.config["hsv_lower"]), np.asarray(self.config["hsv_upper"]))
        kernel = np.ones((self.config["kernel_size"],) * 2, np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.pipeline_pub.publish(Point(x=float("nan"), y=float("nan"), z=0.0))
            self.angle_pub.publish(Float32(data=float("nan")))
            return
        contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if cv2.contourArea(contour) < self.config["min_area"] or not moments["m00"]:
            self.pipeline_pub.publish(Point(x=float("nan"), y=float("nan"), z=0.0))
            self.angle_pub.publish(Float32(data=float("nan")))
            return
        cx, cy = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
        height, width = frame.shape[:2]
        self.pipeline_pub.publish(Point(x=(cx - width / 2) / (width / 2),
                                        y=(cy - height / 2) / (height / 2), z=0.0))
        vx, vy, _, _ = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        raw_angle = math.degrees(math.atan2(float(vy), float(vx)))
        angle = (90 - abs(raw_angle)) * (-1 if raw_angle < 0 else 1)
        self.angle_pub.publish(Float32(data=float(angle)))
        if visualization is not None:
            cv2.drawContours(visualization, [contour], -1, (0, 255, 255), 2)
