#!/usr/bin/env python3
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ==========================================================
# IMPORTING THE EXACT DREADNOUGHT VISION LOGIC
# ==========================================================
from mira2_perception.contour import _getFinalContours
from mira2_perception.rect_fix import correct_rectangle_contour
from mira2_perception.geometry import find_rectangle_corners

# ==========================================================
# SENIOR's ENHANCEMENT FUNCTION
# ==========================================================
def enhance_image(frame):
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    b, g, r = cv2.split(img_rgb)
    b = cv2.normalize(b, None, 0, 255, cv2.NORM_MINMAX)
    g = cv2.normalize(g, None, 0, 255, cv2.NORM_MINMAX)
    r = cv2.normalize(r, None, 0, 255, cv2.NORM_MINMAX)
    img_corrected = cv2.merge((b, g, r))

    lab = cv2.cvtColor(img_corrected, cv2.COLOR_RGB2Lab)
    l_channel, a_channel, b_channel = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(10, 50))
    cl = clahe.apply(l_channel)

    lab_corrected = cv2.merge((cl, a_channel, b_channel))
    return cv2.cvtColor(lab_corrected, cv2.COLOR_Lab2BGR)

class BPNode(Node):
    def __init__(self):
        super().__init__("bp_node")

        # --- NEW: Declare the enhancement parameter (Defaults to True) ---
        self.declare_parameter("enable_enhancement", True)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(PoseStamped, "gate_pose", qos_profile)

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gate_real_width_meters = 1.5   # Adjust to actual gate size

        # Load calibration
        calib_path = os.path.expanduser("~/auv_ws/src/bp_vision/bp_vision/optimized_calib.npz")
        self.is_calibrated = False
        try:
            data = np.load(calib_path) 
            self.camera_matrix = data["mtx"]
            self.dist_coeffs = data["dist"]
            self.focal_length_px = self.camera_matrix[0, 0] 
            self.is_calibrated = True
            self.get_logger().info(f"✅ Calibration loaded! Real focal length: {self.focal_length_px:.2f}px")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Calibration file not found at {calib_path}. Bypassing for prototype phase.")
            self.camera_matrix = None
            self.dist_coeffs = None
            self.focal_length_px = 600.0 # Fallback guess
            self.is_calibrated = False

        # Load the test video using FFmpeg
        video_path = os.path.expanduser("~/auv_ws/src/bp_vision/bp_vision/test.MP4")
        self.cap = cv2.VideoCapture(video_path, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Camera failed to open")
        else:
            self.get_logger().info("✅ Camera opened")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ No frame from camera (or video ended). Looping...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) 
            return

        # ==========================================================
        # 1. Undistort the frame FIRST (if calibrated)
        # ==========================================================
        if self.is_calibrated:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)

        # ==========================================================
        # 2. Check Parameter & Apply Enhancement
        # ==========================================================
        enable_enhancement = self.get_parameter("enable_enhancement").value
        
        if enable_enhancement:
            frame = enhance_image(frame)
            cv2.putText(frame, "Enhancement: ON", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Enhancement: OFF", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # ==========================================================
        # 3. Convert to Grayscale & Process Contours
        # ==========================================================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        gate_pixel_width = 0.0
        lateral_error_px = 0.0
        distance = -1.0
        center_x, center_y = 0, 0

        try:
            # Get raw contours (handles split gates)
            final_contours = _getFinalContours(gray, frame, quality=255, viz=False)

            if final_contours and len(final_contours) > 0:
                # Merge split contours and fix warped 90-degree corners
                corrected_rect_contour = correct_rectangle_contour(final_contours)
                
                if corrected_rect_contour is not None and len(corrected_rect_contour) > 0:
                    points = corrected_rect_contour.reshape(-1, 2)
                    corners_dict = find_rectangle_corners(points)
                    
                    # Extract precise width using top corners
                    tl = corners_dict['top_left']
                    tr = corners_dict['top_right']
                    gate_pixel_width = float(np.linalg.norm(tr - tl))
                    
                    # Calculate center point for lateral error
                    center_x = int(np.mean([p[0] for p in points]))
                    center_y = int(np.mean([p[1] for p in points]))
                    
                    image_center_x = frame.shape[1] / 2.0
                    lateral_error_px = image_center_x - center_x

                    # --- Visualizations ---
                    cv2.polylines(frame, [corrected_rect_contour], isClosed=True, color=(0, 255, 0), thickness=2)
                    for name, pt in corners_dict.items():
                        cv2.circle(frame, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
                    cv2.circle(frame, (center_x, center_y), 4, (255, 165, 0), -1)
                    cv2.putText(frame, f"Width: {int(gate_pixel_width)}px", (int(tl[0]), int(tl[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Lat Err: {lateral_error_px:.1f}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        except Exception as e:
            self.get_logger().warn(f"Vision processing bypassed: {e}")

        # ==========================================================
        # 4. Distance Math & Publishing
        # ==========================================================
        if gate_pixel_width > 0:
            distance = (self.focal_length_px * self.gate_real_width_meters) / gate_pixel_width
            cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        else:
            distance = -1.0  

        cv2.imshow("Dreadnought Vision Pipeline", frame)
        cv2.waitKey(1)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        
        msg.pose.position.x = float(distance)
        msg.pose.position.y = float(lateral_error_px) 
        msg.pose.position.z = 0.0

        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BPNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down BP node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
