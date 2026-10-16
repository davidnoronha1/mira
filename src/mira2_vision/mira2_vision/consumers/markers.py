from utils.parameters import Setting, declare_config, positive
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from custom_msgs.msg import ArucoPose
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from mira2_vision.plugin import Plugin


class Markers(Plugin):
    requires = ('aruco',)
    provides = ()

    def __init__(self, node):
        super().__init__(node)
        self.config = declare_config(node, {
            'marker_size': Setting(0.15, positive),
            'publish_tf': True,
        }, namespace='markers')
        half = self.config['marker_size'] / 2
        self.marker_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
        self.markers_pub = node.create_publisher(ArucoPose, 'markers', 10)
        self.marker_ids_pub = node.create_publisher(String, 'pipeline/markers', 10)
        self.tf_broadcaster = TransformBroadcaster(node) if self.config['publish_tf'] else None

    def process(self, context):
        corners, visible = context.products['aruco']
        self.marker_ids_pub.publish(String(data=','.join(map(str, visible))))
        matrix, header, visualization = context.matrix, context.header, context.preview
        if matrix is None:
            return
        for image_corners, marker_id in zip(corners, visible):
            ok, rvec, tvec = cv2.solvePnP(self.marker_points, image_corners, matrix,
                                       self.node.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                pose = self.make_pose(header, rvec, tvec)
                self.markers_pub.publish(ArucoPose(pose=pose, marker_id=marker_id))
                if self.tf_broadcaster is not None:
                    transform = TransformStamped(header=pose.header)
                    transform.child_frame_id = f"{pose.header.frame_id}/aruco_{marker_id}"
                    transform.transform.translation.x = pose.pose.position.x
                    transform.transform.translation.y = pose.pose.position.y
                    transform.transform.translation.z = pose.pose.position.z
                    transform.transform.rotation = pose.pose.orientation
                    self.tf_broadcaster.sendTransform(transform)
                if visualization is not None:
                    cv2.drawFrameAxes(visualization, matrix, self.node.dist_coeffs, rvec, tvec,
                                     self.config["marker_size"] / 2)

    @staticmethod
    def make_pose(header, rvec, tvec):
        pose = PoseStamped(header=header)
        values = np.asarray(tvec).flatten()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, values)
        quaternion = Rotation.from_rotvec(np.asarray(rvec).flatten()).as_quat()
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quaternion)
        return pose
