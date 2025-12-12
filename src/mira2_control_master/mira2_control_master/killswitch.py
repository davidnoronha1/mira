import rclpy
from rclpy.node import Node
from gpiozero import Button
from custom_msgs.msg import EmergencyKill

class KillSwitchPublisher(Node):
    def __init__(self):
        super().__init__('kill_switch_publisher')
        self.publisher_ = self.create_publisher(EmergencyKill, "/emergency_stop", 10)
        
        # --- THE FIX IS HERE ---
        # bounce_time=0.1 means: "Once the state changes, ignore noise for 0.1 seconds"
        self.button = Button(17, pull_up=True, bounce_time=0.1)
        
        self.button.when_pressed = self.publish_safe_state
        self.button.when_released = self.publish_kill_state
        
        self.get_logger().info("Reed Switch Node Ready (Debounced)")
        
        # Initial check
        if self.button.is_pressed:
            self.publish_safe_state()
        else:
            self.publish_kill_state()

    def publish_safe_state(self):
        msg = EmergencyKill()
        msg.kill_switch = False
        self.publisher_.publish(msg)
        self.get_logger().info("Magnet Detected: System SAFE (Run)")

    def publish_kill_state(self):
        msg = EmergencyKill()
        msg.kill_switch = True
        self.publisher_.publish(msg)
        self.get_logger().warn("Magnet Removed: KILL SIGNAL SENT")

def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchPublisher()
    node.get_logger().info("Kill Switch Script Enabled!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
