#!/usr/bin/env python3
"""Publish fake Detection2DArray messages to /vision/detections for testing."""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2DArray, Detection2D,
    ObjectHypothesisWithPose, BoundingBox2D,
    Pose2D,
)


class FakeDetectionPublisher(Node):
    def __init__(self):
        super().__init__('fake_detection_publisher')
        self.pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz
        self.get_logger().info('Publishing fake detections to /vision/detections at 10 Hz')

    def publish(self):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        d = Detection2D()
        d.header = msg.header
        d.id = 'dock'  # <-- this is what BT nodes match on

        bb = BoundingBox2D()
        bb.center = Pose2D(x=320.0, y=240.0, theta=0.0)
        bb.size_x = 200.0
        bb.size_y = 150.0
        d.bbox = bb

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'dock'
        hyp.hypothesis.score = 0.95
        d.results.append(hyp)

        msg.detections.append(d)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
