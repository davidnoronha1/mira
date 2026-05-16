#!/usr/bin/env python3
"""
commands_normaliser — bridge node for the normalized command interface.

Subscribes to /master/commands_normalized (CommandsNormalized, fields in -1..1),
converts each motion field to PWM microseconds, and republishes to
/master/commands (Commands) — the topic alt_master already consumes.

This lets new code work in the normalized -1..1 range while old code keeps
publishing raw PWM Commands directly. Both feed the same alt_master → Pixhawk
pipeline, so the two can run simultaneously during migration.
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import Commands, CommandsNormalized

_PWM_NEUTRAL = 1500
_PWM_SCALE = 400  # -1 → 1100, 0 → 1500, +1 → 1900


def _to_pwm(value: float) -> int:
    """Convert a normalized command (-1..1) to a PWM value (1100..1900)."""
    clamped = max(-1.0, min(1.0, value))
    return int(_PWM_NEUTRAL + clamped * _PWM_SCALE)


class CommandsNormaliser(Node):
    def __init__(self):
        super().__init__("commands_normaliser")
        self.pub = self.create_publisher(Commands, "/master/commands", 10)
        self.create_subscription(
            CommandsNormalized, "/master/commands_normalized", self.callback, 10
        )
        self.get_logger().info(
            "commands_normaliser running: /master/commands_normalized -> /master/commands"
        )

    def callback(self, msg: CommandsNormalized):
        out = Commands()
        # arm and mode pass through unchanged
        out.arm = msg.arm
        out.mode = msg.mode
        # motion fields: normalized -1..1 -> PWM 1100..1900
        out.forward = _to_pwm(msg.forward)
        out.lateral = _to_pwm(msg.lateral)
        out.thrust = _to_pwm(msg.thrust)
        out.pitch = _to_pwm(msg.pitch)
        out.roll = _to_pwm(msg.roll)
        out.yaw = _to_pwm(msg.yaw)
        out.servo1 = _to_pwm(msg.servo1)
        out.servo2 = _to_pwm(msg.servo2)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CommandsNormaliser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
