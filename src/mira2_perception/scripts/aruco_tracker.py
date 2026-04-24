#!/usr/bin/env python3
"""Detect a target ArUco marker from an RTSP stream and publish its pose and alignment error."""

import os
import threading

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header

MARKER_LENGTH = 0.15  # metres

# --- ArUco setup ---
# Using DICT_ARUCO_ORIGINAL as requested
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

if hasattr(cv2.aruco, "ArucoDetector"):
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

    def detect_markers(frame):
        return DETECTOR.detectMarkers(frame)
else:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    def detect_markers(frame):
        return cv2.aruco.detectMarkers(
            frame,
            ARUCO_DICT,
            parameters=ARUCO_PARAMS
        )

OBJ_POINTS = np.array([
    [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
    [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
], dtype=np.float32)


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # Updated target_id to 28 as requested
        self.target_id = self.declare_parameter('target_id', 28).value
        self.rtsp_url = self.declare_parameter(
            'rtsp_url', 'rtsp://192.168.2.6:2000/image_rtsp').value
        self.visualize = self.declare_parameter('visualize', True).value

        # Load calibration
        calib_path = './optimized_calib.npz'
        if not os.path.exists(calib_path):
            self.get_logger().error(f"Calibration file not found at {calib_path}")
            raise FileNotFoundError(calib_path)
            
        calib = np.load(calib_path)
        self.camera_matrix = calib['mtx'].astype(np.float64)
        self.dist_coeffs = calib['dist'].astype(np.float64)

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.error_pub = self.create_publisher(Vector3, '/aruco/error', 10)

        self.get_logger().info(
            f'Tracking ArUco ID {self.target_id} from {self.rtsp_url} | visualize={self.visualize}')

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open RTSP stream: {self.rtsp_url}')
            return

        self.get_logger().info('RTSP stream opened.')

        while not self._stop.is_set() and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn('Lost RTSP stream, retrying...')
                cap.release()
                cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                cv2.waitKey(1000)
                continue

            corners, ids, _ = detect_markers(frame)

            if ids is not None:
                if self.visualize:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id != self.target_id:
                        continue

                    ok, rvec, tvec = cv2.solvePnP(
                        OBJ_POINTS, corners[i], self.camera_matrix,
                        self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    if not ok:
                        continue

                    cv2.solvePnPRefineLM(
                        OBJ_POINTS, corners[i], self.camera_matrix,
                        self.dist_coeffs, rvec, tvec)

                    # CRITICAL FIX: Flatten tvec to 1D array to avoid Deprecation and TypeErrors
                    tvec = tvec.flatten()
                    rvec_f = rvec.flatten()
                    
                    rot = Rotation.from_rotvec(rvec_f)
                    quat = rot.as_quat() # Result is [x, y, z, w]

                    # --- Publish pose ---
                    msg = PoseStamped()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera'
                    msg.pose.position.x = float(tvec[0])
                    msg.pose.position.y = float(tvec[1])
                    msg.pose.position.z = float(tvec[2])
                    msg.pose.orientation.x = float(quat[0])
                    msg.pose.orientation.y = float(quat[1])
                    msg.pose.orientation.z = float(quat[2])
                    msg.pose.orientation.w = float(quat[3])
                    self.pose_pub.publish(msg)

                    # --- Publish error ---
                    err = Vector3()
                    err.x = float(tvec[0])
                    err.y = float(tvec[1])
                    err.z = 0.0
                    self.error_pub.publish(err)

                    # --- Visualization ---
                    if self.visualize:
                        cv2.drawFrameAxes(
                            frame, self.camera_matrix, self.dist_coeffs,
                            rvec, tvec, MARKER_LENGTH * 0.75
                        )

                        # FIX: Now tvec[0] is a scalar, so formatting works
                        text = f"ID:{marker_id} x:{tvec[0]:.2f} y:{tvec[1]:.2f} z:{tvec[2]:.2f}"
                        pt = tuple(corners[i][0][0].astype(int))
                        cv2.putText(frame, text, pt,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 2)
                    break

            if self.visualize:
                cv2.imshow("Aruco Tracker", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

        cap.release()
        cv2.destroyAllWindows()

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Prevent double shutdown error
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
