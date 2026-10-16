from utils.depth import associate_depth, associate_frame_depth
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from mira2_vision.plugin import Plugin
from utils.parameters import Setting, declare_config, string_list, positive


class DepthTargets(Plugin):
    requires = ('detections_2d',)
    provides = ('depth_association',)

    def __init__(self, node):
        super().__init__(node)
        self.config = declare_config(node, {
            'target_class': 'blue_bucket',
            'min_depth': Setting(0.1, positive),
            'max_depth': Setting(20.0, positive),
            'frame_classes': Setting(['gate', 'qual gate'], string_list, dynamic_typing=True),
        }, namespace='depth_targets')
        if self.config['min_depth'] >= self.config['max_depth']:
            raise ValueError('depth_targets.min_depth must be below max_depth')
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
            is_frame = any(r.hypothesis.class_id in self.config['frame_classes'] for r in detection.results)
            associate = associate_frame_depth if is_frame else associate_depth
            xyz = associate(depth,
                                  (box.center.position.x * width, box.center.position.y * height,
                                   box.size_x * width, box.size_y * height), matrix,
                                  self.config["min_depth"], self.config["max_depth"])
            if xyz is None:
                continue
            for result in detection.results:
                result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = xyz
                result.pose.pose.orientation.w = 1.0
                if result.hypothesis.class_id == self.config["target_class"]:
                    if best is None or result.hypothesis.score > best[0]:
                        best = (result.hypothesis.score, result.pose.pose, detection.bbox, result.hypothesis.class_id)
        if best is not None:
            self.target_pose_pub.publish(PoseStamped(header=header, pose=best[1]))
            self.target_point_pub.publish(Point(x=2 * best[2].center.position.x - 1,
                                                y=2 * best[2].center.position.y - 1,
                                                z=1.0 / best[1].position.z))
            self.target_color_pub.publish(String(data=best[3].removesuffix("_bucket")))
