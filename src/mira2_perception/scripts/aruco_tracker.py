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
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

OBJ_POINTS = np.array([
    [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
    [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
], dtype=np.float32)


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        self.target_id = self.declare_parameter('target_id', 0).value
        self.rtsp_url = self.declare_parameter(
            'rtsp_url', 'rtsp://192.168.2.6:2002/image_rtsp').value

        # Load calibration from package share
        pkg_share = get_package_share_directory('mira2_perception')
        calib_path = os.path.join(pkg_share, 'optimized_calib.npz')
        calib = np.load(calib_path)
        self.camera_matrix = calib['mtx'].astype(np.float64)
        self.dist_coeffs = calib['dist'].astype(np.float64)

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        # x = lateral error (camera X), y = forward error (camera Y)
        # Both zero when the marker is centred under the camera.
        self.error_pub = self.create_publisher(Vector3, '/aruco/error', 10)

        self.get_logger().info(
            f'Tracking ArUco ID {self.target_id} from {self.rtsp_url}')

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
                continue

            corners, ids, _ = DETECTOR.detectMarkers(frame)
            if ids is None:
                continue

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

                rot = Rotation.from_rotvec(rvec.flatten())
                quat = rot.as_quat()  # x, y, z, w

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

                # Publish alignment error: x = lateral, y = forward.
                # Setpoint for both is 0 (marker centred under camera).
                err = Vector3()
                err.x = float(tvec[0])  # lateral offset in camera frame
                err.y = float(tvec[1])  # forward offset in camera frame
                err.z = 0.0
                self.error_pub.publish(err)

                break  # only one target marker

        cap.release()

    def destroy_node(self):
        self._stop.set()
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
