"""Simulation-only detector: exact 2D boxes from Gazebo's boundingbox_camera.

A drop-in for YOLO in Gazebo (same detections_2d product), so missions can be
exercised before a model exists for the simulated objects. The sensor must
share the RGB camera's pose and intrinsics; objects are tagged with integer
labels by the Label system in the world file, and gazebo_boxes.classes maps
each label (list index) to the class name YOLO would use.
"""
from collections import deque

from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from utils import topics
from utils.parameters import Setting, declare_config, nonnegative, string_list
from mira2_vision.plugin import Plugin


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class GazeboBoxes(Plugin):
    requires = ()
    provides = ('detections_2d',)

    def __init__(self, node):
        super().__init__(node)
        if not self.settings['simulation']:
            raise ValueError('gazebo_boxes reads simulator ground truth; use yolo on hardware')
        self.config = declare_config(node, {
            'topic': topics.FRONT_BOXES,
            'classes': Setting([''], string_list, dynamic_typing=True),
            'min_area': Setting(16.0, nonnegative),  # pixels^2; drops sliver boxes at frame edges
        }, namespace='gazebo_boxes')
        self.recent = deque(maxlen=8)
        self.unknown = set()
        self.subscription = node.create_subscription(
            Detection2DArray, self.config['topic'], lambda msg: self.recent.append(msg),
            qos_profile_sensor_data)

    def process(self, context):
        target = stamp_ns(context.header)
        skew = int(self.settings['depth_max_skew'] * 1e9)
        boxes = next((m for m in reversed(self.recent) if abs(stamp_ns(m.header) - target) <= skew), None)
        if boxes is None:
            return  # no matching render yet; consumers skip this frame
        height, width = context.image.shape[:2]
        detections = Detection2DArray(header=context.header)
        for box in boxes.detections:
            if box.bbox.size_x * box.bbox.size_y < self.config['min_area'] or not box.results:
                continue
            label = box.results[0].hypothesis.class_id
            name = self._class_name(label)
            if name is None:
                continue
            detection = Detection2D(header=context.header, id=name)
            detection.bbox.center.position.x = box.bbox.center.position.x / width
            detection.bbox.center.position.y = box.bbox.center.position.y / height
            detection.bbox.size_x = box.bbox.size_x / width
            detection.bbox.size_y = box.bbox.size_y / height
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        context.products['detections_2d'] = detections

    def _class_name(self, label):
        try:
            name = self.config['classes'][int(label)]
        except (ValueError, IndexError):
            name = ''
        if not name and label not in self.unknown:
            self.unknown.add(label)
            self.node.get_logger().warn(f'gazebo_boxes: label {label} has no entry in gazebo_boxes.classes')
        return name or None

    def close(self):
        self.node.destroy_subscription(self.subscription)
