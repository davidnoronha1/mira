#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os
import sys
from scipy.spatial.transform import Rotation as R, Slerp
# print(sys.path)
from utils.lib.coordinate import opencv_to_ros

# ==========================================
# 1. SMART POSE FILTER (Outlier + Smoothing)
# ==========================================
class PoseFilter:
    def __init__(self, alpha_pos=0.6, alpha_rot=0.6, max_jump_dist=0.5):
        self.alpha_pos = alpha_pos
        self.alpha_rot = alpha_rot
        self.max_jump = max_jump_dist
        
        self.prev_pos = None
        self.prev_quat = None
        
        # Buffer for median filtering (size 3 ignores 1-frame glitches without lag)
        self.pos_buffer = []
        self.buffer_size = 3
        
        self.consecutive_rejections = 0
        self.max_rejections_before_reset = 5 # Blind for max ~0.2s

    def update(self, curr_pos, curr_quat):
        if self.prev_pos is None:
            self.prev_pos, self.prev_quat = curr_pos, curr_quat
            return curr_pos, curr_quat

        # --- JUMP REJECTION ---
        dist = np.linalg.norm(curr_pos - self.prev_pos)
        
        # If detection jumps too far, reject it UNLESS we've been blind too long
        if dist > self.max_jump and self.consecutive_rejections < self.max_rejections_before_reset:
            self.consecutive_rejections += 1
            return None, None 
        
        # If we reach here, reset rejection counter
        # If we were blind too long, we jump-sync (alpha=1.0) to reacquire
        effective_alpha = 1.0 if self.consecutive_rejections >= self.max_rejections_before_reset else self.alpha_pos
        self.consecutive_rejections = 0

        # --- MEDIAN STABILIZATION ---
        self.pos_buffer.append(curr_pos)
        if len(self.pos_buffer) > self.buffer_size:
            self.pos_buffer.pop(0)
        median_pos = np.median(np.array(self.pos_buffer), axis=0)

        # --- EMA SMOOTHING ---
        filt_pos = effective_alpha * median_pos + (1 - effective_alpha) * self.prev_pos
        try:
            rots = R.from_quat([self.prev_quat, curr_quat])
            slerp = Slerp([0, 1], rots)
            filt_quat = slerp([self.alpha_rot])[0].as_quat()
        except:
            filt_quat = curr_quat

        self.prev_pos, self.prev_quat = filt_pos, filt_quat
        return filt_pos, filt_quat

    def reset(self):
        self.prev_pos = None
        self.pos_buffer = []
        self.consecutive_rejections = 0

# ==========================================
# 2. BOARD MAPPING (X-Right, Y-Down)
# ==========================================
DEFAULT_MARKER_MAP = {
    28: [-0.29, -0.49, 0.0],  # Top-Left (-X, -Y)
    7:  [ 0.29, -0.49, 0.0],  # Top-Right (+X, -Y)
    19: [-0.29,  0.49, 0.0],  # Bot-Left (-X, +Y)
    96: [ 0.29,  0.49, 0.0]   # Bot-Right (+X, +Y)
}

class UnderwaterDockingNode(Node):
    def __init__(self):
        super().__init__('underwater_docking_node')
        
        # Parameters
        self.declare_parameter('calibration_file', 'calibration_data.json')
        self.declare_parameter('video_file', '') 
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('enable_gui', True)

        # Setup Calib
        calib_path = self.get_parameter('calibration_file').value
        self.mtx, self.dist = self.load_calibration(calib_path)
        self.marker_size = self.get_parameter('marker_size').value
        
        # ArUco Config
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.params = aruco.DetectorParameters()
        self.params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        
        # Enhancement Filter (CLAHE)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        
        # Pose Filter
        self.filter = PoseFilter(alpha_pos=0.6, alpha_rot=0.6, max_jump_dist=0.6)
        
        # Pre-calc 3D Points
        self.board_points = self.generate_board_points()

        # Video Source
        video_path = "rtsp://192.168.2.6:8554/image_rtsp"# self.get_parameter('video_file').value
        if video_path:
            self.get_logger().info(f"Using Video: {video_path}")
            self.cap = cv2.VideoCapture(video_path)
            self.is_live = False
        else:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.is_live = True

        # ROS Setup
        self.publisher_ = self.create_publisher(PoseStamped, 'dock_pose', 10)
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.gui = self.get_parameter('enable_gui').value

    def load_calibration(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Calibration file not found: {path}")
            sys.exit(1)
        with open(path, 'r') as f:
            data = json.load(f)
        return np.array(data['camera_matrix'], dtype=np.float32), \
               np.array(data['dist_coeff'], dtype=np.float32)

    def generate_board_points(self):
        pts = {}
        s = self.marker_size / 2.0
        # Standard ArUco Winding: TL, TR, BR, BL
        base = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float32)
        for mid, offset in DEFAULT_MARKER_MAP.items():
            pts[mid] = base + np.array(offset, dtype=np.float32)
        return pts

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            if not self.is_live: self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # --- UNDERWATER IMAGE PROCESSING ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray) # Sharpens edges in murky water

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)

        if ids is not None:
            obj_pts, img_pts = [], []
            for i, mid in enumerate(ids.flatten()):
                if mid in self.board_points:
                    obj_pts.append(self.board_points[mid])
                    img_pts.append(corners[i][0])

            if len(obj_pts) > 0:
                # SolvePnP treats all markers as ONE rigid board
                success, rvec, tvec = cv2.solvePnP(
                    np.vstack(obj_pts), np.vstack(img_pts), self.mtx, self.dist
                )
                
                if success:
                    raw_q = R.from_matrix(cv2.Rodrigues(rvec)[0]).as_quat()
                    f_pos, f_quat = self.filter.update(tvec.flatten(), raw_q)

                    if f_pos is not None:
                        # PUBLISH POSE
                        msg = Point()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "camera_bottom"
                        msg.x = float(f_pos[0])
                        msg.y = float(f_pos[1])
                        msg.z = float(f_pos[1.0/f_pos[2]]) if f_pos[2] > 0 else 0.0 # Invert Z for better control response
                        self.publisher_.publish(msg)

                        if self.gui:
                            cv2.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, 0.4)
                            # Draw center dot
                            c_img, _ = cv2.projectPoints(np.array([[0,0,0]], dtype=np.float32), rvec, tvec, self.mtx, self.dist)
                            cv2.circle(frame, tuple(c_img[0].ravel().astype(int)), 10, (0, 255, 0), -1)
        else:
            # If markers lost entirely, count towards a filter reset
            self.filter.consecutive_rejections += 1
            if self.filter.consecutive_rejections > 25: # 1 second
                self.filter.reset()

        if self.gui:
            cv2.imshow('Robust Underwater Docking', frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
