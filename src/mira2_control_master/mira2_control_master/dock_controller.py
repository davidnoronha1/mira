#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import Commands, Telemetry
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# --- Enum for States ---
class State:
    SEARCH = 0  # surges forward with a rpm of 1600 can be changed in search subroutine
    ALIGN_XY = 1 # align acc to dock's center in the x - y axis 
    ALIGN_YAW = 2 # align using the yaw only 
    APPROACH = 3  # descent towards the dock
    BLIND_LATCH = 4  # when the bot is close enough to the dock 
    DOCKED = 5 # will know if puck is connected
    RECOVERY = 6

class DockingController(Node):
    def __init__(self):
        super().__init__("docking_controller")

        # --- Tunable PID Parameters ---
        self.kp_sway = 200.0  # PWM counts per meter error
        self.kp_surge = 150.0 
        self.kp_yaw = 300.0   # PWM counts per radian error
        
        # --- Config ---
        self.pwm_neutral = 1500
        self.pwm_range = 400 # +/- 400 from neutral
        self.search_surge_pwm = 1600 # Approx 0.3 m/s
        self.latch_duration = 4.0 # Seconds to push blindly
        
        # --- State Variables ---
        self.state = State.SEARCH
        self.last_pose_time = 0.0
        self.dock_visible = False
        self.target_pose = None # [x, y, z, roll, pitch, yaw] relative to camera
        self.current_heading = 0.0
        self.blind_timer_start = None

        # --- ROS Interfaces ---
        self.cmd_pub = self.create_publisher(Commands, "/master/commands", 10)
        
        '''self.create_subscription(PoseStamped, "dock_pose", self.pose_callback, 10)'''
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
self.create_subscription(PoseStamped, "dock_pose", self.pose_callback, qos_profile)
        self.create_subscription(Telemetry, "/master/telemetry", self.telem_callback, 10)

        # Main Control Loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Docking Controller Initialized in SEARCH mode.")

    # --- Callbacks ---

    def telem_callback(self, msg):
        self.current_heading = msg.heading

    def pose_callback(self, msg):
        self.last_pose_time = time.time()
        self.dock_visible = True
        
        # Convert Quaternion to Euler (Yaw specifically)
        # Assuming pose is Camera -> Dock transform
        q = msg.pose.orientation
        (roll, pitch, yaw) = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # Store relative pose: x (right), y (down), z (forward) - Standard Camera Frame
        # Adjust depending on your specific camera frame setup!
        self.target_pose = {
            'x': msg.pose.position.x, # Horizontal Error
            'y': msg.pose.position.y, # Vertical Error (Depth diff)
            'z': msg.pose.position.z, # Distance
            'yaw': yaw
        }

    # --- Main Loop ---

    def control_loop(self):
        cmd = Commands()
        cmd.mode = "ALT_HOLD" # Ensure we are in depth hold mode
        cmd.arm = 1
        
        # Reset channels to neutral
        cmd.pitch = 1500
        cmd.roll = 1500
        cmd.thrust = 1500
        cmd.yaw = 1500
        cmd.forward = 1500
        cmd.lateral = 1500

        # Check visibility timeout (Lost marker)
        if time.time() - self.last_pose_time > 1.0:
            self.dock_visible = False

        # --- STATE MACHINE ---

        if self.state == State.SEARCH:
            # Phase 1: Search
            cmd.forward = self.search_surge_pwm # Surge 0.3
            self.get_logger().info("Phase 1: Searching...", throttle_duration_sec=1)
            
            if self.dock_visible:
                self.get_logger().info("Dock Found! Switching to Align XY.")
                self.state = State.ALIGN_XY

        elif self.state == State.ALIGN_XY:
            # Phase 2: Alignment
            if not self.dock_visible:
                self.state = State.SEARCH #Lost it,go back
                return

            # PID Control
            err_sway = self.target_pose['x']
            # Note: For Surge in this phase, we want to maintain a specific distance 
            
            cmd.lateral = self.apply_pid(err_sway, self.kp_sway)
            cmd.forward = 1500 # Very slow approach while aligning
            
            # Check Alignment
            if abs(err_sway) < 0.1: # 10cm threshold
                self.get_logger().info("XY Aligned. Switching to Yaw.")
                self.state = State.ALIGN_YAW

        elif self.state == State.ALIGN_YAW:
            # Phase 3: Yaw
            if not self.dock_visible:
                self.state = State.SEARCH
                return
            err_yaw = self.target_pose['yaw']
            # Simple P-Controller for Yaw
            # Note: 1500 + output. Positive output = Clockwise usually.
            cmd.yaw = self.apply_pid(err_yaw, self.kp_yaw)
            
            # Stop movement to focus on rotation
            cmd.forward = 1500
            cmd.lateral = 1500

            if abs(err_yaw) < 0.1: # ~5 degrees
                self.get_logger().info("Yaw Aligned. Switching to Approach.")
                self.state = State.APPROACH

        elif self.state == State.APPROACH:
            # Phase 4: Approach
            
            # Blind Spot Detection
            if not self.dock_visible and self.target_pose['z'] < 0.5:
                self.get_logger().info("Entering Blind Spot Latch Sequence!")
                self.blind_timer_start = time.time()
                self.state = State.BLIND_LATCH
                return
            elif not self.dock_visible:
                self.state = State.SEARCH # Lost it far away
                return

            # Active Approach
            err_sway = self.target_pose['x']
            cmd.lateral = self.apply_pid(err_sway, self.kp_sway)
            #cmd.forward = 1575 # Approach speed
            cmd.thrust = 1575
        elif self.state == State.BLIND_LATCH:
            # Phase 4b: Blind Push
            elapsed = time.time() - self.blind_timer_start
            
            cmd.forward = 1600 # Push firm
            cmd.lateral = 1500 # Lock sway
            cmd.yaw = 1500     # Lock yaw (Gyro will hold heading)
            
            if elapsed > self.latch_duration:
                self.get_logger().info("Latch Timer Complete. Checking status...")
                self.state = State.DOCKED # Or RECOVERY based on sensor

        elif self.state == State.DOCKED:
            # Phase 5: Success
            cmd.forward = 0
            cmd.lateral = 0
            cmd.thrust = 0
            cmd.arm = 0 # Disarm
            self.get_logger().info("DOCKED.")
            
        elif self.state == State.RECOVERY:
            # Phase 5b: Retry
            # going to implement the "Up 1m and Back" logic here 
            pass

        self.cmd_pub.publish(cmd)

    # --- Helpers ---
    
    def apply_pid(self, error, kp):
        output = int(error * kp)
        # Clamp
        output = max(min(output, self.pwm_range), -self.pwm_range)
        return self.pwm_neutral + output

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
    node = DockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
