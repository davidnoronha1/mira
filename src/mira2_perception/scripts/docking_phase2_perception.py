#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os
import sys
from scipy.spatial.transform import Rotation as R, Slerp

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
        
        self.pos_buffer = []
        self.buffer_size = 3
        
        self.consecutive_rejections = 0
        self.max_rejections_before_reset = 5 

    def update(self, curr_pos, curr_quat):
        if self.prev_pos is None:
            self.prev_pos, self.prev_quat = curr_pos, curr_quat
            return curr_pos, curr_quat

        dist = np.linalg.norm(curr_pos - self.prev_pos)
        
        if dist > self.max_jump and self.consecutive_rejections < self.max_rejections_before_reset:
            self.consecutive_rejections += 1
            return None, None 
        
        effective_alpha = 1.0 if self.consecutive_rejections >= self.max_rejections_before_reset else self.alpha_pos
        self.consecutive_rejections = 0

        self.pos_buffer.append(curr_pos)
        if len(self.pos_buffer) > self.buffer_size:
            self.pos_buffer.pop(0)
        median_pos = np.median(np.array(self.pos_buffer), axis=0)

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
    28: [-0.29, -0.49, 0.0],  
    7:  [ 0.29, -0.49, 0.0],  
    19: [-0.29,  0.49, 0.0],  
    96: [ 0.29,  0.49, 0.0]   
}

class UnderwaterDockingNode(Node):
    def __init__(self):
        super().__init__('underwater_docking_node')
        
        self.declare_parameter('calibration_file', 'calibration_data.json')
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('enable_gui', True)
        self.declare_parameter('image_topic', '/bluerov2/camera_bottom/image_color')  # NEW PARAM

        calib_path = self.get_parameter('calibration_file').value
        self.mtx, self.dist = self.load_calibration(calib_path)
        self.marker_size = self.get_parameter('marker_size').value
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.params = aruco.DetectorParameters()
        self.params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 23
        self.params.adaptiveThreshWinSizeStep = 10
        
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.params)
        
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        self.filter = PoseFilter(alpha_pos=0.6, alpha_rot=0.6, max_jump_dist=0.6)
        self.board_points = self.generate_board_points()

        # ✅ ROS2 IMAGE SUBSCRIBER
        self.bridge = CvBridge()
        topic = self.get_parameter('image_topic').value

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Subscribed to image topic: {topic}")

        self.publisher_ = self.create_publisher(PoseStamped, 'dock_pose', 10)
        self.gui = self.get_parameter('enable_gui').value

    def load_calibration(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Calibration file not found: {path}")
            return np.eye(3), np.zeros(5)
        with open(path, 'r') as f:
            data = json.load(f)
        return np.array(data['camera_matrix'], dtype=np.float32), \
               np.array(data['dist_coeff'], dtype=np.float32)

    def generate_board_points(self):
        pts = {}
        s = self.marker_size / 2.0
        base = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float32)
        for mid, offset in DEFAULT_MARKER_MAP.items():
            pts[mid] = base + np.array(offset, dtype=np.float32)
        return pts

    # ✅ REPLACED timer_callback → image_callback
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray) 

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            obj_pts, img_pts = [], []
            for i, mid in enumerate(ids.flatten()):
                if mid in self.board_points:
                    obj_pts.append(self.board_points[mid])
                    img_pts.append(corners[i][0])

            if len(obj_pts) > 0:
                success, rvec, tvec = cv2.solvePnP(
                    np.vstack(obj_pts), np.vstack(img_pts), self.mtx, self.dist
                )
                
                if success:
                    raw_q = R.from_matrix(cv2.Rodrigues(rvec)[0]).as_quat()
                    f_pos, f_quat = self.filter.update(tvec.flatten(), raw_q)

                    if f_pos is not None:
                        msg_out = PoseStamped()
                        msg_out.header.stamp = self.get_clock().now().to_msg()
                        msg_out.header.frame_id = "camera_optical_frame"
                        msg_out.pose.position.x, msg_out.pose.position.y, msg_out.pose.position.z = f_pos.astype(float)
                        msg_out.pose.orientation.x, msg_out.pose.orientation.y, msg_out.pose.orientation.z, msg_out.pose.orientation.w = f_quat.astype(float)
                        self.publisher_.publish(msg_out)

                        if self.gui:
                            filt_rot_matrix = R.from_quat(f_quat).as_matrix()
                            f_rvec, _ = cv2.Rodrigues(filt_rot_matrix)
                            f_tvec = np.array(f_pos).reshape(3, 1)

                            cv2.drawFrameAxes(frame, self.mtx, self.dist, f_rvec, f_tvec, 0.15)
                            
                            c_img, _ = cv2.projectPoints(
                                np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
                                f_rvec, f_tvec, self.mtx, self.dist
                            )
                            cv2.circle(frame, tuple(c_img[0].ravel().astype(int)), 10, (0, 255, 0), -1)
        else:
            self.filter.consecutive_rejections += 1
            if self.filter.consecutive_rejections > 25:
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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()