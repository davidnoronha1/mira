import time
import cv2
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from scipy.spatial.transform import Rotation, Slerp
from mira2_vision.plugin import Plugin


class DockingBoard(Plugin):
    requires = ('aruco',)
    provides = ()

    def __init__(self, node):
        super().__init__(node)
        half = self.settings["marker_size"] / 2
        board_base = np.array([[-half, -half, 0], [half, -half, 0],
                               [half, half, 0], [-half, half, 0]], np.float32)
        offsets = np.asarray(self.settings["board_offsets"], np.float32).reshape(-1, 3)
        self.board_points = {mid: board_base + offset for mid, offset in zip(self.settings["board_ids"], offsets)}
        self.previous_board = None
        self.board_rejections = 0

        self.board_pub = node.create_publisher(PoseStamped, "dock_pose", 10)
        self.dock_target_pub = node.create_publisher(Point, "dock_target", 10)

    def process(self, context):
        if context.matrix is None:
            return
        corners, visible = context.products['aruco']
        height, width = context.image.shape[:2]
        self.frame_dimensions = (width, height)
        self.process_board(corners, visible, context.matrix, context.header)

    def process_board(self, corners, ids, matrix, header):
        object_points, image_points = [], []
        for image_corners, marker_id in zip(corners, ids):
            if marker_id in self.board_points:
                object_points.append(self.board_points[marker_id])
                image_points.append(image_corners.reshape(4, 2))
        if not object_points:
            return
        ok, rvec, tvec = cv2.solvePnP(np.vstack(object_points), np.vstack(image_points),
                                    matrix, self.node.dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return
        position = tvec.flatten()
        if not np.isfinite(position).all() or position[2] <= 0:
            return
        quaternion = Rotation.from_rotvec(rvec.flatten()).as_quat()
        now = time.monotonic()
        if self.previous_board is not None:
            old_position, old_quaternion, old_time = self.previous_board
            if now - old_time < self.settings["max_frame_age"]:
                if np.linalg.norm(position - old_position) > self.settings["pose_filter_max_jump"] and \
                        self.board_rejections < self.settings["pose_filter_reset_after"]:
                    self.board_rejections += 1
                    return
                if not self.board_rejections:
                    alpha = self.settings["pose_filter_alpha"]
                    position = alpha * position + (1 - alpha) * old_position
                    quaternion = Slerp([0, 1], Rotation.from_quat([old_quaternion, quaternion]))([alpha])[0].as_quat()
        self.previous_board = (position, quaternion, now)
        self.board_rejections = 0
        pose = PoseStamped(header=header)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quaternion)
        self.board_pub.publish(pose)
        # Existing BT alignment/descent nodes use image-centered offsets and
        # inverse range, not PoseStamped. Keep the two interfaces distinct.
        image_center, _ = cv2.projectPoints(np.zeros((1, 3), np.float32), np.zeros(3),
                                          position, matrix, self.node.dist_coeffs)
        u, v = image_center.reshape(2)
        width, height = self.frame_dimensions
        self.dock_target_pub.publish(Point(x=float((u - width / 2) / (width / 2)),
                                           y=float((v - height / 2) / (height / 2)),
                                           z=float(1.0 / position[2])))
