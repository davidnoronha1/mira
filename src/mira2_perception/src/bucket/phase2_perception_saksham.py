#!/usr/bin/env python3

import cv2
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from collections import deque

from custom_msgs.msg import _2DObject

class RTSPCapture:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                self.frame = frame
    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.ret else None
    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()

class BucketPerception(Node):
    def __init__(self):
        super().__init__("bucket_perception_node")

        self.target_pub = self.create_publisher(_2DObject, "bucket_target_2d", 10)

        self.get_logger().info("Connecting to stream...")
        #self.camera = RTSPCapture("rtsp://192.168.2.6:8554/image_rtsp")
        #self.camera = cv2.VideoCapture("vid1.mov")
        #change it according to the needs 
        self.LOWER_BLUE = np.array([105, 40, 50])
        self.UPPER_BLUE = np.array([125, 255, 255])
        
        #orange HSV
        self.LOWER_ORANGE = np.array([0, 50, 50])
        self.UPPER_ORANGE = np.array([30, 255, 255])
        self.MIN_BUCKET_AREA = 3000

        smooth_n = 5
        self.cx_hist = deque(maxlen=smooth_n)
        self.cy_hist = deque(maxlen=smooth_n)
        self.size_hist = deque(maxlen=smooth_n)

        self.create_timer(0.05, self.process_frame)

    def is_valid_ellipse(self, ellipse, frame_shape):
        (cx, cy), (ma, Mi), angle = ellipse
        h, w = frame_shape[:2]

        if not (0 < cx < w and 0 < cy < h):
            return False
        if ma < 10 or Mi < 10:
            return False
        ratio = max(ma, Mi) / (min(ma, Mi) + 1e-5)
        if ratio > 4.0:
            return False

        return True

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret or frame is None:
            return
        h, w = frame.shape[:2]
        frame_cx, frame_cy = w // 2, h // 2

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # creating individual color masks
        mask_blue = cv2.inRange(frame_hsv, self.LOWER_BLUE, self.UPPER_BLUE)
        mask_orange = cv2.inRange(frame_hsv, self.LOWER_ORANGE, self.UPPER_ORANGE)
        # combine them to find ANY bucket shape
        combined_mask = cv2.bitwise_or(mask_blue, mask_orange)

        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_small)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel_large)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vis = frame.copy()
        detected = False
        detected_color = None
        ellipse = None

        if contours:
            valid_contours = [c for c in contours if cv2.contourArea(c) > self.MIN_BUCKET_AREA]
            
            if valid_contours:
                largest = max(valid_contours, key=cv2.contourArea)
                
                if len(largest) >= 5:
                    candidate = cv2.fitEllipse(largest)
                    
                    if self.is_valid_ellipse(candidate, frame.shape):
                        ellipse = candidate
                        (cx, cy), (ma, Mi), angle = ellipse
                        roi = np.zeros_like(combined_mask)
                        cv2.drawContours(roi, [largest], -1, 255, -1)
                        
                        b_px = cv2.countNonZero(cv2.bitwise_and(mask_blue, roi))
                        o_px = cv2.countNonZero(cv2.bitwise_and(mask_orange, roi))

                        if b_px > o_px:
                            detected_color = "blue"
                        else:
                            detected_color = "orange"

                        self.cx_hist.append(int(cx))
                        self.cy_hist.append(int(cy))
                        self.size_hist.append(max(ma, Mi))
                        detected = True

        if self.cx_hist:
            smooth_cx = int(np.mean(self.cx_hist))
            smooth_cy = int(np.mean(self.cy_hist))
            smooth_size = np.mean(self.size_hist)

            offset_x = smooth_cx - frame_cx
            offset_y = smooth_cy - frame_cy
            norm_x = offset_x / (w / 2)
            norm_y = offset_y / (h / 2)
            depth_proxy = smooth_size / w

            if detected:
                obj_msg = _2DObject()
                obj_msg.point.x = float(norm_x)
                obj_msg.point.y = float(norm_y)
                obj_msg.point.z = float(depth_proxy)
                obj_msg.id = detected_color
                self.target_pub.publish(obj_msg)

            if ellipse is not None: 
                color = (255, 0, 0) if detected_color == "blue" else (0, 165, 255)
                cv2.ellipse(vis, ellipse, color, 2)

            cv2.circle(vis, (smooth_cx, smooth_cy), 8, (0, 255, 0), -1)
            cv2.drawMarker(vis, (frame_cx, frame_cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.line(vis, (frame_cx, frame_cy), (smooth_cx, smooth_cy), (0, 255, 255), 2)

            status = f"DETECTED: {detected_color.upper()}" if detected else "LOST (coasting)"
            status_color = (0, 255, 0) if detected else (0, 165, 255)

            cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(vis, f"Offset norm: ({norm_x:+.2f}, {norm_y:+.2f})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, f"Depth proxy: {depth_proxy:.3f}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.drawMarker(vis, (frame_cx, frame_cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(vis, "NO DETECTION", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        #cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Bucket Vision", vis)
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    node = BucketPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.camera.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()