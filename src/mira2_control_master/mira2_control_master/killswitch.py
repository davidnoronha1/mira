import rclpy
from rclpy.node import Node
import lgpio
import time

from custom_msgs.msg import EmergencyKill


GPIO_CHIP = 0     # gpiochip0
GPIO_PIN = 17    # BCM 17 (physical pin 11)


class KillSwitchPublisher(Node):
    def __init__(self):
        super().__init__('kill_switch_publisher')

        self.publisher_ = self.create_publisher(
            EmergencyKill,
            "/esp/telemetry",
            10
        )

        # Open GPIO chip
        self.h = lgpio.gpiochip_open(GPIO_CHIP)

        # Input with pull-up (same as gpiozero Button(pull_up=True))
        lgpio.gpio_claim_input(
            self.h,
            GPIO_PIN,
            lgpio.SET_PULL_UP
        )

        self.last_level = lgpio.gpio_read(self.h, GPIO_PIN)

        self.get_logger().info("Kill switch armed (lgpio)")

        # Poll GPIO (simple + reliable)
        self.timer = self.create_timer(0.02, self.poll_gpio)  # 50 Hz

    def poll_gpio(self):
        level = lgpio.gpio_read(self.h, GPIO_PIN)

        # Detect RELEASE: 0 -> 1
        if self.last_level == 0 and level == 1:
            self.send_kill()

        self.last_level = level

    def send_kill(self):
        msg = EmergencyKill()
        msg.kill_switch = True
        self.publisher_.publish(msg)
        self.get_logger().warn("Sending EmergencyKill signal.")

    def destroy_node(self):
        try:
            lgpio.gpiochip_close(self.h)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
