#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Char


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("key_mapper")
    pub = node.create_publisher(Char, "/keys", 10)
    while rclpy.ok():
        c = Char()
        # print("Send input:")
        try:
            input_char = input()
            if len(input_char) == 1:
                c.data = ord(input_char)
                pub.publish(c)
            else:
                node.get_logger().info("Please input only one character.")
        except (EOFError, KeyboardInterrupt):
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
