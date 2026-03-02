#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import serial

class KillSwitchPublisher(Node):
    is_killed = False
    def __init__(self):
        super().__init__('kill_switch_pub_node')
        
        # Create service client for toggle emergency
        self.toggle_client = self.create_client(Trigger, '/toggle_emergency')
        while not self.toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /toggle_emergency service...')
        
        # Connect to Arduino
        try:
            self.ser = serial.Serial('/dev/Arduino', 9600, timeout=1)
            self.ser.flush()
            self.get_logger().info("Connected to Arduino on /dev/Arduino")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        # Check often (0.01s) to catch the signal immediately
        print("Starting loop")
        self.timer = self.create_timer(0.01, self.check_serial)

    def check_serial(self):
        if self.ser.in_waiting > 0:
            try:
                # Read the line and remove whitespace/newlines
                line = self.ser.readline().decode('utf-8').strip()

                if not self.is_killed and line.strip() == "0":
                    self.get_logger().warn("Magnet Removed  -> Calling toggle service")
                    request = Trigger.Request()
                    self.toggle_client.call_async(request)
                    self.is_killed = True
                elif self.is_killed and line.strip() == "1":
                    self.get_logger().warn("Magent Attached -> Calling toggle service")
                    request = Trigger.Request()
                    self.toggle_client.call_async(request)
                    self.is_killed = False

                    # We use warn so it shows up yellow in the logs

            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
