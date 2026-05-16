#!/usr/bin/env python3

'''
Changes/Todo

1) Replace with actual pid values for yaw, lateral
2) Change surge speed 
3) Add Search and turning at the end 
'''

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float32

from custom_msgs.msg import Commands
from custom_msgs.msg import Telemetry


class PipelineControlsNode(Node):
    def __init__(self):
        super().__init__('pipeline_controls_node')

        # PID Values
        self.kp_lateral = 150
        self.kp_yaw = 100
        self.kp_surge = 100

        self.kd_lateral = 0
        self.kd_yaw = 0
        self.kd_surge = 0

        # Offset and Angle init
        self.angle = None
        self.norm_angle = None
        self.x_offset = None
        self.y_offset = None
        self.lateral_active = False
        self.yaw_active = False
        self.surge_active = False
        self.max_pwm = 1700
        self.min_pwm = 1300

        # Startup state
        self.startup_done = False
        self.startup_state = 0
        self.startup_time = None

        self.cmd_msg = Commands()
        self.cmd_msg.arm = False
        self.cmd_msg.mode = "MANUAL"
        self.cmd_msg.forward = 1500
        self.cmd_msg.lateral = 1500
        self.cmd_msg.thrust = 1500
        self.cmd_msg.yaw = 1500
        self.cmd_msg.pitch = 1500
        self.cmd_msg.roll = 1500

        # Publishers
        self.cmd_pub = self.create_publisher(Commands, '/master/commands', 10)

        # Subscribers
        self.telemetry_sub = self.create_subscription(Telemetry, '/master/telemetry', self.telemetry_callback, 10)
        self.offset_sub = self.create_subscription(Point, '/pipeline/offset', self.offset_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/pipeline/angle', self.angle_callback, 10)

        # Timers
        self.create_timer(0.1, self.control_loop)

    def startup_sequence(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.startup_state == 0:
            self.get_logger().info("Disarming...")
            self.cmd_msg.arm = False
            self.cmd_pub.publish(self.cmd_msg)
            self.startup_time = now
            self.startup_state = 1

        elif self.startup_state == 1 and (now - self.startup_time) >= 2.0:
            self.get_logger().info("Setting mode to MANUAL...")
            self.cmd_msg.mode = "MANUAL"
            self.cmd_pub.publish(self.cmd_msg)
            self.startup_time = now
            self.startup_state = 2

        elif self.startup_state == 2 and (now - self.startup_time) >= 2.0:
            self.get_logger().info("Arming...")
            self.cmd_msg.arm = True
            self.cmd_pub.publish(self.cmd_msg)
            self.startup_done = True
            self.get_logger().info("Startup complete.")

    # Callbacks
    def telemetry_callback(self, msg):
        return

    def angle_callback(self, msg):
        self.angle = msg.data
        self.norm_angle = -(self.angle / 90)

    def offset_callback(self, msg):
        self.x_offset = msg.x
        self.y_offset = msg.y

    def control_loop(self):
        if not self.startup_done:
            self.startup_sequence()
            return

        if self.norm_angle is None or self.x_offset is None or self.y_offset is None:
            self.get_logger().info(f"angle: {None}, x_offset: {self.x_offset}, y_offset: {self.y_offset}")
            return
        
        if (abs(self.angle) > 10):
            self.yaw_active = True
            self.get_logger().info("Aligning Yaw")

        elif (abs(self.angle) <=5):
            self.yaw_active = False
            self.cmd_msg.yaw = 1500
            self.get_logger().info("Aligned Yaw")

        if self.yaw_active:
            self.align_yaw()

        else:
            if abs(self.x_offset) > 0.1:
                self.lateral_active = True
                self.get_logger().info("Aligning Lateral")
            elif abs(self.x_offset) < 0.05:
                self.lateral_active = False
                self.get_logger().info("Aligned Lateral")

            if self.lateral_active:
                self.align_x()

            # else:
            #     if abs(self.y_offset) > 0.1:
            #         self.surge_active = True
            #         self.get_logger().info("Aligning Surge")
            #     elif abs(self.y_offset) < 0.05:
            #         self.surge_active = False
            #         self.get_logger().info("Aligned Surge")

            #     if self.surge_active:
            #         self.align_y()
                
            else:
                self.surge()


        self.get_logger().info(f"Commands -> yaw: {self.cmd_msg.yaw}, lateral: {self.cmd_msg.lateral}, surge: {self.cmd_msg.forward}")
        self.cmd_pub.publish(self.cmd_msg)



    def align_yaw(self):
        self.cmd_msg.arm = True
        yaw_cmd = int(1500 + (self.kp_yaw * self.norm_angle))
        yaw_cmd = self.clamp(yaw_cmd)
        self.cmd_msg.yaw = yaw_cmd
        self.cmd_msg.forward = 1500
        self.cmd_msg.lateral = 1500

    def surge(self):
        self.cmd_msg.arm = True
        self.cmd_msg.forward = 1620
        self.cmd_msg.lateral = 1500
        self.cmd_msg.yaw = 1500


    def align_x(self):
        self.cmd_msg.arm = True
        lateral_cmd = int(1500 + (self.kp_lateral * self.x_offset))
        lateral_cmd = self.clamp(lateral_cmd)
        self.cmd_msg.lateral = lateral_cmd
        self.cmd_msg.forward = 1500
        self.cmd_msg.yaw = 1500

    # def align_y(self):
    #     self.cmd_msg.arm = True
    #     forward_cmd = int(1500 + (self.kp_surge * -self.y_offset))
    #     forward_cmd = self.clamp(forward_cmd)
    #     self.cmd_msg.forward = forward_cmd
    #     self.cmd_msg.lateral = 1500
    #     self.cmd_msg.yaw = 1500
        




    def clamp(self, pwm):
        if pwm <= 1500:
            max_pwm = 1460
            min_pwm = self.min_pwm
        else:
            max_pwm = self.max_pwm
            min_pwm = 1540

        return min(max(pwm, min_pwm), max_pwm)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineControlsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()