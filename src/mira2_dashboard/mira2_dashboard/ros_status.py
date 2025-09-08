import rclpy
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import String   # replace with the correct type for your topic!

from PyQt6.QtCore import qInfo  # Import qInfo for logging

def is_node_alive(target_node: str) -> bool:
	"""Check if a ROS2 node with the given name is alive."""
	
	node = Node("node_checker")
	try:
		node_names = node.get_node_names()
		status = target_node in node_names
		qInfo(f"Node '{target_node}' alive: {status}")
		return status
	finally:
		node.destroy_node()
		# rclpy.shutdown()

def is_topic_alive(topic_name: str, msg_type=String, timeout: float = 5.0) -> bool:
	"""
	Check if a ROS2 topic is alive by subscribing and waiting for one message.
	msg_type must be the correct ROS2 message type for the topic.
	"""
	rclpy.init(args=None)
	node = Node("topic_checker")
	future = Future()

	def callback(msg):
		if not future.done():
			future.set_result(True)

	sub = node.create_subscription(msg_type, topic_name, callback, 10)

	try:
		start = node.get_clock().now()
		while rclpy.ok() and not future.done():
			rclpy.spin_once(node, timeout_sec=0.1)
			elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
			if elapsed > timeout:
				qInfo(f"Topic '{topic_name}' alive: False (timeout)")
				return False
		status = future.result() if future.done() else False
		qInfo(f"Topic '{topic_name}' alive: {status}")
		return status
	finally:
		node.destroy_subscription(sub)
		node.destroy_node()
		rclpy.shutdown()
