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
        calib_path = os.path.join(
            get_package_share_directory('mira2_perception'), 'optimized_calib.npz')
        if not os.path.exists(calib_path):
            self.get_logger().error(f"Calibration file not found at {calib_path}")
            raise FileNotFoundError(calib_path)
            
        calib = np.load(calib_path)
        self.camera_matrix = calib['mtx'].astype(np.float64)
        self.dist_coeffs = calib['dist'].astype(np.float64)

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.error_pub = self.create_publisher(Vector3, '/aruco/error', 10)

        self._last_error = None  # (x, y) tuple; held when marker leaves frame

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

            # --- Underwater image enhancement ---
            # Denoise to reduce temporal flickering
            frame = cv2.bilateralFilter(frame, d=7, sigmaColor=50, sigmaSpace=50)
            # CLAHE on L channel to boost contrast in murky water
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            l = clahe.apply(l)
            frame = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
            # Sharpen edges to make marker borders crisper
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
            frame = cv2.filter2D(frame, -1, kernel)

            corners, ids, _ = detect_markers(frame)

            frame_h, frame_w = frame.shape[:2]
            frame_cx, frame_cy = frame_w // 2, frame_h // 2
            marker_center = None
            display_err_x, display_err_y = None, None

            if ids is not None:
                detected = ids.flatten().tolist()
                self.get_logger().info(f'Detected IDs: {detected} (target={self.target_id})')
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

                    self._last_error = (float(tvec[0]), float(tvec[1]))
                    display_err_x = float(tvec[0])
                    display_err_y = float(tvec[1])

                    # --- Visualization ---
                    if self.visualize:
                        cv2.drawFrameAxes(
                            frame, self.camera_matrix, self.dist_coeffs,
                            rvec, tvec, MARKER_LENGTH * 0.75
                        )

                        text = f"ID:{marker_id} x:{tvec[0]:.2f} y:{tvec[1]:.2f} z:{tvec[2]:.2f}"
                        pt = tuple(corners[i][0][0].astype(int))
                        cv2.putText(frame, text, pt,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 2)

                        # Pixel centre of the detected marker
                        cx = int(corners[i][0][:, 0].mean())
                        cy = int(corners[i][0][:, 1].mean())
                        marker_center = (cx, cy)

                    break

            # Failsafe: if marker not detected this frame, republish last known error
            if display_err_x is None and self._last_error is not None:
                err = Vector3()
                err.x = self._last_error[0]
                err.y = self._last_error[1]
                err.z = 0.0
                self.error_pub.publish(err)
                display_err_x, display_err_y = self._last_error

            if self.visualize:
                # Draw crosshair at frame centre
                cv2.drawMarker(frame, (frame_cx, frame_cy), (0, 0, 255),
                               cv2.MARKER_CROSS, 20, 2)

                if marker_center is not None:
                    # Line from marker centre to frame centre
                    cv2.line(frame, marker_center, (frame_cx, frame_cy),
                             (0, 255, 255), 2)
                    cv2.circle(frame, marker_center, 5, (0, 255, 255), -1)

                if display_err_x is not None:
                    # Error overlay in top-left corner
                    cv2.putText(frame,
                                f"err_x: {display_err_x:+.3f} m",
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 255, 255), 2)
                    cv2.putText(frame,
                                f"err_y: {display_err_y:+.3f} m",
                                (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 255, 255), 2)

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
