from utils.depth import associate_depth
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from mira2_vision.plugin import Plugin


class DepthTargets(Plugin):
    requires = ('detections_2d',)
    provides = ('depth_association',)

    def __init__(self, node):
        super().__init__(node)
        self.target_point_pub = node.create_publisher(Point, "target_point", 1)
        self.target_color_pub = node.create_publisher(String, "target_color", 1)
        self.target_pose_pub = node.create_publisher(PoseStamped, "target_pose", 1)

    def process(self, context):
        detections = context.products['detections_2d']
        if context.matrix is not None and context.depth is not None:
            height, width = context.image.shape[:2]
            self.associate_targets(detections, context.matrix, width, height, context.depth.image)
        context.products['depth_association'] = detections

    def associate_targets(self, detections, matrix, width, height, depth):
        header = detections.header
        best = None
        for detection in detections.detections:
            box = detection.bbox
            # Shared 2D contract stores boxes normalized to image dimensions.
            xyz = associate_depth(depth,
                                  (box.center.position.x * width, box.center.position.y * height,
                                   box.size_x * width, box.size_y * height), matrix,
                                  self.settings["depth_min"], self.settings["depth_max"])
            if xyz is None:
                continue
            for result in detection.results:
                result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = xyz
                result.pose.pose.orientation.w = 1.0
                if result.hypothesis.class_id == self.settings["target_class"]:
                    if best is None or result.hypothesis.score > best[0]:
                        best = (result.hypothesis.score, result.pose.pose, detection.bbox, result.hypothesis.class_id)
        if best is not None:
            self.target_pose_pub.publish(PoseStamped(header=header, pose=best[1]))
            self.target_point_pub.publish(Point(x=2 * best[2].center.position.x - 1,
                                                y=2 * best[2].center.position.y - 1,
                                                z=1.0 / best[1].position.z))
            self.target_color_pub.publish(String(data=best[3].removesuffix("_bucket")))
