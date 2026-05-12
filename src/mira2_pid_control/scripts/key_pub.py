#!/usr/bin/python3
"""
DEPRECATED: This script is deprecated and may be removed in a future release.

This script publishes keyboard input as ROS messages. For regular Python input
handling without ROS integration, use:

    input_char = input("Enter a character: ")
    if len(input_char) == 1:
        print(f"You entered: {input_char} (ASCII: {ord(input_char)})")
    else:
        print("Please input only one character.")

For non-blocking input, consider using the 'readchar' package:
    pip install readchar
    import readchar
    char = readchar.readchar()
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Char


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("key_mapper")
    node.get_logger().warn(
        "DEPRECATED: key_pub.py is deprecated and may be removed in a future release. "
        "For regular Python input: input_char = input(). "
        "For non-blocking input, use: pip install readchar && import readchar"
    )
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
