#!/usr/bin/env python3
"""Mirror the Arduino magnetic kill switch to the master's emergency service."""

import time
import serial
import rclpy

from rclpy.node import Node
from std_srvs.srv import SetBool
from utils import topics


class KillSwitchPublisher(Node):
    SERIAL_DEVICE = "/dev/Arduino"
    BAUDRATE = 9600
    EMERGENCY_SERVICE = topics.EMERGENCY_STOP

    def __init__(self):
        super().__init__("kill_switch_pub_node")

        self.emergency_client = self.create_client(SetBool, self.EMERGENCY_SERVICE)

        self.get_logger().info(f"Waiting for {self.EMERGENCY_SERVICE}")
        if not self.emergency_client.wait_for_service():
            raise RuntimeError("Shutdown while waiting for emergency service")

        self.confirmed_killed = None

        self.ser = serial.Serial(self.SERIAL_DEVICE, self.BAUDRATE, timeout=1)

        self.ser.reset_input_buffer()
        self.get_logger().info("Connected to Arduino")

    def run(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode("utf-8").strip()

                if line not in {"0", "1"}:
                    time.sleep(0.01)
                    continue

                killed = line == "0"

                if killed != self.confirmed_killed:
                    request = SetBool.Request()
                    request.data = killed

                    future = self.emergency_client.call_async(request)

                    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

                    if not future.done():
                        self.emergency_client.remove_pending_request(future)
                        self.get_logger().error("Emergency service timeout")
                    else:
                        response = future.result()

                        if response.success:
                            self.confirmed_killed = killed
                            state = "engaged" if killed else "cleared"
                            self.get_logger().warn(
                                f"Emergency stop {state}: {response.message}"
                            )
                        else:
                            self.get_logger().error(
                                f"Master rejected emergency update: {response.message}"
                            )

                time.sleep(0.01)

            except (serial.SerialException, UnicodeDecodeError) as error:
                self.get_logger().error(f"Serial error: {error}")
                break

            except Exception as error:
                self.get_logger().error(f"Service error: {error}")
                time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = KillSwitchPublisher()
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.ser.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
