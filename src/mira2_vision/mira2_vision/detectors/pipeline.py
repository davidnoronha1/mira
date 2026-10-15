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
        self.pipeline_pub = node.create_publisher(Point, 'pipeline/offset', 10)
        self.angle_pub = node.create_publisher(Float32, 'pipeline/angle', 10)

    def process(self, context):
        frame, visualization = context.image, context.preview
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.asarray(self.settings["hsv_lower"]), np.asarray(self.settings["hsv_upper"]))
        kernel = np.ones((self.settings["pipeline_kernel_size"],) * 2, np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.pipeline_pub.publish(Point(x=float("nan"), y=float("nan"), z=0.0))
            self.angle_pub.publish(Float32(data=float("nan")))
            return
        contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if cv2.contourArea(contour) < self.settings["pipeline_min_area"] or not moments["m00"]:
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
