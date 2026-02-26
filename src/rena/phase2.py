#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import Commands, Telemetry
from std_msgs.msg import String
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class State:
    SEARCH = 0
    ALIGN_XY = 1
    APPROACH = 2
    LOCK = 3
    SEARCH2 = 4


class BucketControls(Node):

    def __init__(self):
        super().__init__("bucket_control2")

        #pid
        self.kp_sway = 200.0
        self.kp_surge = 150.0
        self.kp_yaw = 300.0

        # config
        self.pwm_neutral = 1500
        self.pwm_range = 400
        self.search_surge_pwm = 1600
        self.lower_to_bucket = None  

        # State Variables
        self.state = State.SEARCH
        self.last_pose_time = 0.0
        self.bucket_visible = False
        self.bucket_blue = False
        self.bucket_orange = False
        self.target_pose = None
        self.current_heading = 0.0
        self.blind_timer_start = None
        # 1 oe -1 for direction
        self.search2_direction = 1
        self.search2_start = None
        self.search2_timer = 3.0

        # ROS
        self.cmd_pub = self.create_publisher(
            Commands, "/master/commands", 10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            PoseStamped,
            "ellipsoid_pose",
            self.pose_callback,
            qos_profile
        )

        self.create_subscription(
            Telemetry,
            "/master/telemetry",
            self.telem_callback,
            10
        )

        #add one more subscription according to name in perception node for colour of bucket
        self.create_subscription(
            String,
            "bucket_clr",
            self.buck_callback,
            10
        )

        #loop
        self.create_timer(0.05, self.control_loop)

    #callbacks

    def telem_callback(self, msg):
        self.current_heading = msg.heading

    def pose_callback(self, msg):
        self.last_pose_time = time.time()
        self.bucket_visible = True

        self.target_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
        }

    def buck_callback(self, msg):
        if msg.data == "blue":
            self.bucket_blue = True
            self.bucket_orange = False
        elif msg.data == "orange":
            self.bucket_blue = False
            self.bucket_orange = True

    # ---------------- Main Loop ----------------

    def control_loop(self):

        cmd = Commands()
        cmd.mode = "ALT_HOLD"
        cmd.arm = 1

        cmd.pitch = 1500
        cmd.roll = 1500
        cmd.thrust = 1500
        cmd.yaw = 1500
        cmd.forward = 1500
        cmd.lateral = 1500

        if time.time() - self.last_pose_time > 1.0:
            self.bucket_visible = False

        # -------- STATE MACHINE --------

        if self.state == State.SEARCH:

            cmd.forward = self.search_surge_pwm
            self.get_logger().info(
                "Phase 1: Searching...", throttle_duration_sec=1
            )

            if self.bucket_visible and self.bucket_blue:
                self.get_logger().info("correct bucket found -> align xy")
                self.state = State.ALIGN_XY

            elif self.bucket_visible and not self.bucket_blue:
                self.get_logger().info("wrong bucket -> restarting search")
                self.search2_start = time.time()
                self.search2_direction = 1
                self.state = State.SEARCH2


        elif self.state == State.ALIGN_XY:

            if not self.bucket_visible:
                self.state = State.SEARCH
                return

            err_x = self.target_pose['x']
            err_y = self.target_pose['y']

            cmd.lateral = self.apply_pid(err_x, self.kp_sway)
            cmd.forward = self.apply_pid(err_y, self.kp_surge)

            self.log_motion(cmd)

            cmd.thrust = 1500

            if abs(err_x) < 0.1 and abs(err_y) < 0.1:
                self.get_logger().info(
                    "XY aligned over bucket. Beginning descent."
                )
                self.state = State.APPROACH

        elif self.state == State.APPROACH:

            if (
                not self.bucket_visible
                and self.target_pose
                and self.target_pose['z'] < 0.5
            ):
                self.get_logger().info(
                    "Bucket lost at close range — locking position"
                )
                self.blind_timer_start = time.time()
                self.state = State.LOCK
                return

            elif not self.bucket_visible:
                self.state = State.SEARCH
                return

            err_x = self.target_pose['x']
            err_y = self.target_pose['y']

            cmd.lateral = self.apply_pid(err_x, self.kp_sway)
            cmd.forward = self.apply_pid(err_y, self.kp_surge)

            if self.target_pose['z'] > 1.0:
                cmd.thrust = 1560
            else:
                cmd.thrust = 1530

        elif self.state == State.LOCK:

            elapsed = time.time() - self.blind_timer_start

            cmd.forward = 1500
            cmd.lateral = 1500
            cmd.thrust = 1500
            cmd.yaw = 1500

            if elapsed > 3.0:
                self.get_logger().info("Position locked over bucket")
                
# this is the sideway search for other buckets
        elif self.state == State.SEARCH2:

            if self.bucket_visible and self.bucket_blue:
                self.get_logger().info("correct bucket found -> align XY")
                self.state = State.ALIGN_XY
                return
            
            if self.bucket_visible and not self.bucket_blue:
                self.get_logger().info("wrong bucket -> secondary search again")
            
            sway_pwm = 1500 + (self.search2_direction * 50)
            cmd.lateral = sway_pwm
            cmd.yaw = 1500 #just in case to keep it neutral

            elapsed= time.time() - self.search2_start

            if elapsed > self.search2_timer:

                self.search2_direction = self.search2_direction * -1
                self.get_logger().info("reversing direction, no buckets")
                self.search2_start = time.time()
            
        self.cmd_pub.publish(cmd)

    #Helpers

    def log_motion(self, cmd):
        if cmd.lateral > 1500:
            self.get_logger().info("Sway RIGHT", throttle_duration_sec=0.5)
        elif cmd.lateral < 1500:
            self.get_logger().info("Sway LEFT", throttle_duration_sec=0.5)

        if cmd.forward > 1500:
            self.get_logger().info("Surge FORWARD", throttle_duration_sec=0.5)
        elif cmd.forward < 1500:
            self.get_logger().info("Surge BACKWARD", throttle_duration_sec=0.5)

        if cmd.thrust > 1500:
            self.get_logger().info("Heave UP/DOWN active", throttle_duration_sec=0.5)


    def apply_pid(self, error, kp):
        output = int(error * kp)
        output = max(min(output, self.pwm_range), -self.pwm_range)
        return self.pwm_neutral + output

    def euler_from_quaternion(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


def main(args=None):
    rclpy.init(args=args)
    node = BucketControls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()