#!/usr/bin/env python3
"""
Detect a target ArUco marker from an image source and publish its pose and alignment error.
Prioritizes CameraInfoManager calibration (defaulting to bottomcam.ini in pkg share), 
falling back to a legacy .npz file.
"""

import os
import threading
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from camera_info_manager import CameraInfoManager

# Assuming this utility exists in your workspace
try:
    from utils.image_source import build_image_source
except ImportError:
    def build_image_source(uri, node):
        node.get_logger().info(f"Opening CV VideoCapture for {uri}")
        return cv2.VideoCapture(uri)

MARKER_LENGTH = 0.15  # metres

# --- ArUco setup ---
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

if hasattr(cv2.aruco, "ArucoDetector"):
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
    def detect_markers(frame):
        return DETECTOR.detectMarkers(frame)
else:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
    def detect_markers(frame):
        return cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)

OBJ_POINTS = np.array([
    [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
    [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
], dtype=np.float32)


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # Construct default path to bottomcam.ini
        try:
            pkg_share = get_package_share_directory('mira2_perception')
            default_info_url = f"file://{os.path.join(pkg_share, 'config', 'bottomcam.ini')}"
        except Exception:
            default_info_url = "ros_topic://camera_info"

        # --- Parameters ---
        self.target_id = self.declare_parameter('target_id', 28).value
        self.image_source_uri = self.declare_parameter(
            'image_source', 'rtsp://192.168.2.6:2000/image_rtsp').value
        self.visualize = self.declare_parameter('visualize', True).value
        
        # Calibration Parameters
        self.calib_file_path = self.declare_parameter('calibration_file', '').value
        self.camera_info_url = self.declare_parameter('camera_info_url', default_info_url).value
        self.camera_name = self.declare_parameter('camera_name', 'mira2_camera').value

        # --- Internal State ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self._calib_ready = threading.Event()
        self._last_error = None

        # --- Setup Calibration ---
        self._setup_calibration()

        # --- ROS Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.error_pub = self.create_publisher(Vector3, '/aruco/error', 10)

        self.get_logger().info(
            f'Tracking ArUco ID {self.target_id} | info_url={self.camera_info_url}')

        # --- Processing Thread ---
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _setup_calibration(self):
        """Initializes CameraInfoManager and sets up topic listener."""
        self.info_manager = CameraInfoManager(self, cname=self.camera_name, url=self.camera_info_url)
        
        # 1. Try to load from Manager (works for file URLs like the default .ini)
        if self.info_manager.isCalibrated():
            info = self.info_manager.getCameraInfo()
            self._update_calib_from_msg(info)
            self.get_logger().info(f"Calibration loaded from URL: {self.camera_info_url}")
        else:
            # 2. If URL is ros_topic:// or file load failed, subscribe to topic
            topic_name = 'camera_info'
            if self.camera_info_url.startswith('ros_topic://'):
                topic_name = self.camera_info_url.replace('ros_topic://', '')

            self.info_sub = self.create_subscription(
                CameraInfo, topic_name, self._info_callback, 10)
            self.get_logger().info(f"CameraInfoManager uncalibrated. Listening on '{topic_name}'...")

    def _info_callback(self, msg: CameraInfo):
        """Callback to capture CameraInfo from the ROS topic."""
        if self.camera_matrix is None:
            if len(msg.k) == 9:
                self._update_calib_from_msg(msg)
                self.get_logger().info("Calibration received via ROS topic.")

    def _update_calib_from_msg(self, msg):
        """Extracts matrix and coefficients from ROS message."""
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self._calib_ready.set()

    def _load_from_npz_fallback(self):
        """Loads calibration from the legacy .npz file as a final fallback."""
        if not self.calib_file_path:
            return False

        path = self.calib_file_path
        try:
            if not os.path.exists(path):
                pkg_path = os.path.join(get_package_share_directory('mira2_perception'), path)
                if os.path.exists(pkg_path):
                    path = pkg_path

            calib = np.load(path)
            self.camera_matrix = calib['mtx'].astype(np.float64)
            self.dist_coeffs = calib['dist'].astype(np.float64)
            self.get_logger().info(f"Fallback: Loaded calibration from NPZ: {path}")
            self._calib_ready.set()
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to load fallback NPZ: {e}")
            return False

    def _capture_loop(self):
        """Main processing loop running in a separate thread."""
        if not self._calib_ready.wait(timeout=10.0):
            self.get_logger().warn("Calibration timeout from ROS. Trying NPZ fallback...")
            if not self._load_from_npz_fallback():
                self.get_logger().error("No calibration available. Exiting tracker thread.")
                return

        source = build_image_source(self.image_source_uri, self)
        self.get_logger().info(f'Image source opened: {self.image_source_uri}')

        while not self._stop.is_set() and rclpy.ok():
            if hasattr(source, 'grab'):
                frame = source.grab()
            else:
                ret, frame = source.read()
                if not ret: frame = None

            if frame is None:
                self.get_logger().warn('No frame received, retrying source...')
                if hasattr(source, 'release'): source.release()
                cv2.waitKey(1000)
                source = build_image_source(self.image_source_uri, self)
                continue

            # --- Image Enhancement ---
            frame = cv2.bilateralFilter(frame, d=7, sigmaColor=50, sigmaSpace=50)
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            l = clahe.apply(l)
            frame = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
            frame = cv2.filter2D(frame, -1, kernel)

            corners, ids, _ = detect_markers(frame)

            frame_h, frame_w = frame.shape[:2]
            frame_cx, frame_cy = frame_w // 2, frame_h // 2
            marker_center = None
            display_err_x, display_err_y = None, None

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

                    tvec = tvec.flatten()
                    rvec_f = rvec.flatten()
                    rot = Rotation.from_rotvec(rvec_f)
                    quat = rot.as_quat()

                    now = self.get_clock().now().to_msg()
                    msg = PoseStamped()
                    msg.header.stamp = now
                    msg.header.frame_id = 'camera'
                    msg.pose.position.x = float(tvec[0])
                    msg.pose.position.y = float(tvec[1])
                    msg.pose.position.z = float(tvec[2])
                    msg.pose.orientation.x = float(quat[0])
                    msg.pose.orientation.y = float(quat[1])
                    msg.pose.orientation.z = float(quat[2])
                    msg.pose.orientation.w = float(quat[3])
                    self.pose_pub.publish(msg)

                    err = Vector3()
                    err.x = float(tvec[0])
                    err.y = float(tvec[1])
                    err.z = 0.0
                    self.error_pub.publish(err)

                    self._last_error = (float(tvec[0]), float(tvec[1]))
                    display_err_x, display_err_y = self._last_error

                    if self.visualize:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                          rvec, tvec, MARKER_LENGTH * 0.75)
                        text = f"ID:{marker_id} x:{tvec[0]:.2f} y:{tvec[1]:.2f}"
                        pt = tuple(corners[i][0][0].astype(int))
                        cv2.putText(frame, text, pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        cx = int(corners[i][0][:, 0].mean())
                        cy = int(corners[i][0][:, 1].mean())
                        marker_center = (cx, cy)
                    break

            if display_err_x is None and self._last_error is not None:
                err = Vector3()
                err.x = self._last_error[0]
                err.y = self._last_error[1]
                err.z = 0.0
                self.error_pub.publish(err)
                display_err_x, display_err_y = self._last_error

            if self.visualize:
                cv2.drawMarker(frame, (frame_cx, frame_cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                if marker_center is not None:
                    cv2.line(frame, marker_center, (frame_cx, frame_cy), (0, 255, 255), 2)
                    cv2.circle(frame, marker_center, 5, (0, 255, 255), -1)

                if display_err_x is not None:
                    cv2.putText(frame, f"err_x: {display_err_x:+.3f} m", (10, 25), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                    cv2.putText(frame, f"err_y: {display_err_y:+.3f} m", (10, 52), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

                cv2.imshow("Aruco Tracker", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

        if hasattr(source, 'release'): source.release()
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
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
