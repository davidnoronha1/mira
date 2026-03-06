#!/usr/bin/env python3
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory

# ==========================================================
# IMPORTING THE EXACT DREADNOUGHT VISION LOGIC
# ==========================================================
from mira2_perception.contour import _getFinalContours
from mira2_perception.rect_fix import correct_rectangle_contour
from mira2_perception.geometry import find_rectangle_corners

# ==========================================================
# IMAGE ENHANCEMENT FUNCTION
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

        # --- RECTIFIED: Dynamic Parameters ---
        self.declare_parameter("enable_enhancement", True)
        # Empty string defaults to live camera (Device 0)
        self.declare_parameter("video_path", "") 

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(PoseStamped, "gate_pose", qos_profile)

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gate_real_width_meters = 1.5 

        # --- RECTIFIED: Use Package Share Directory for Calibration ---
        try:
            package_share_dir = get_package_share_directory('mira2_perception')
            calib_path = os.path.join(package_share_dir, 'optimized_calib.npz')
            
            data = np.load(calib_path) 
            self.camera_matrix = data["mtx"]
            self.dist_coeffs = data["dist"]
            self.focal_length_px = self.camera_matrix[0, 0] 
            self.is_calibrated = True
            self.get_logger().info(f"✅ Calibration loaded from share! Focal length: {self.focal_length_px:.2f}px")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Calibration failed to load: {e}. Using fallback focal length.")
            self.camera_matrix = None
            self.dist_coeffs = None
            self.focal_length_px = 600.0 
            self.is_calibrated = False

        # --- RECTIFIED: Video Source via ROS 2 Parameter ---
        video_param = self.get_parameter("video_path").get_parameter_value().string_value
        
        if video_param == "":
            self.cap = cv2.VideoCapture(0)
            self.get_logger().info(" Initialized live camera feed (Device 0)")
        else:
            # Resolve path (handles ~ or relative paths)
            full_video_path = os.path.expanduser(video_param)
            self.cap = cv2.VideoCapture(full_video_path, cv2.CAP_FFMPEG)
            self.get_logger().info(f" Initialized video file: {full_video_path}")

        if not self.cap.isOpened():
            self.get_logger().error("Camera or Video failed to open")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # Loop video if it's a file, otherwise return
            if self.get_parameter("video_path").value != "":
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # 1. Undistort FIRST
        if self.is_calibrated:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)

        # 2. Apply Enhancement based on Parameter
        if self.get_parameter("enable_enhancement").value:
            frame = enhance_image(frame)
            cv2.putText(frame, "Enhancement: ON", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Enhancement: OFF", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 3. Process Contours
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gate_pixel_width = 0.0
        lateral_error_px = 0.0
        distance = -1.0
        center_x, center_y = 0, 0

        try:
            final_contours = _getFinalContours(gray, frame, quality=255, viz=False)

            if final_contours and len(final_contours) > 0:
                corrected_rect_contour = correct_rectangle_contour(final_contours)
                
                if corrected_rect_contour is not None and len(corrected_rect_contour) > 0:
                    points = corrected_rect_contour.reshape(-1, 2)
                    corners_dict = find_rectangle_corners(points)
                    
                    tl = corners_dict['top_left']
                    tr = corners_dict['top_right']
                    gate_pixel_width = float(np.linalg.norm(tr - tl))
                    
                    center_x = int(np.mean([p[0] for p in points]))
                    center_y = int(np.mean([p[1] for p in points]))
                    
                    image_center_x = frame.shape[1] / 2.0
                    lateral_error_px = image_center_x - center_x

                    # Visuals
                    cv2.polylines(frame, [corrected_rect_contour], isClosed=True, color=(0, 255, 0), thickness=2)
                    cv2.putText(frame, f"Lat Err: {lateral_error_px:.1f}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        except Exception as e:
            self.get_logger().debug(f"Vision processing frame skip: {e}")

        # 4. Distance & Publishing
        if gate_pixel_width > 0:
            distance = (self.focal_length_px * self.gate_real_width_meters) / gate_pixel_width
            cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

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
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BPNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
