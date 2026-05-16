#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import Commands, Telemetry
import time

# ==========================================
# ROBUST PID CONTROLLER
# ==========================================
class PID:
    def __init__(self, kp, ki, kd, max_out=400):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
            
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        
        # Clamp output to prevent runaway motors
        return max(min(output, self.max_out), -self.max_out)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ==========================================
# MAIN CONTROLLER NODE
# ==========================================
class Phase2Controller(Node):
    def __init__(self):
        super().__init__("phase2_controls")

        # =========================================================
        # 1. DIRECTION MULTIPLIERS (EASY FLIPPING)
        # =========================================================
        # If the bot moves LEFT when it should move RIGHT, change to -1.
        # If it moves BACKWARD when it should move FORWARD, change to -1.
        self.dir_sway  = 1   # Lateral X (Left/Right)
        self.dir_surge = -1  # Forward Y (Forward/Backward) -> OpenCV Y is usually Down/Back, so -1 is typical
        
        # =========================================================
        # 2. TUNABLE PID PARAMETERS (No Yaw)
        # =========================================================
        self.pid_sway  = PID(kp=200.0, ki=0.0, kd=50.0)
        self.pid_surge = PID(kp=200.0, ki=0.0, kd=50.0)

        # =========================================================
        # 3. CONFIGURATION
        # =========================================================
        self.pwm_neutral = 1500
        self.latch_duration = 10.0      # Seconds to push blindly onto the dock
        self.ascend_duration = 2.0      # Seconds to go UP at the very beginning
        self.ascend_pwm = 1600          # Thrust PWM to go UP (>1500)
        self.blind_spot_z = 0.3         # Meters (when to switch to blind plunge)
        self.xy_tolerance = 0.05        # Meters (5cm alignment tolerance)

        # --- State Variables ---
        self.state = "STARTUP"
        self.last_pose_time = 0.0
        self.dock_visible = False
        
        # Pose data from ArUco (X, Y, Z)
        self.target_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        self.blind_timer_start = 0.0
        self.ascend_timer_start = 0.0
        self.prev_loop_time = time.time()

        # --- Startup Sequence State ---
        self.startup_done = False
        self.startup_state = 0
        self.startup_time = 0.0

        # --- Command Message ---
        self.cmd_msg = Commands()
        self.cmd_msg.mode = "MANUAL" # STRICTLY MANUAL
        self.cmd_msg.arm = False
        self.reset_commands()

        # --- Telemetry state ---
        self.is_armed = False

        # --- ROS Interfaces ---
        self.cmd_pub = self.create_publisher(Commands, "/master/commands", 10)
        self.create_subscription(PoseStamped, "dock_pose", self.pose_callback, 10)
        self.create_subscription(Telemetry, "/master/telemetry", self.telem_callback, 10)

        # Run loop at 20Hz (0.05s)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("=========================================")
        self.get_logger().info("🔥 PHASE 2 CONTROLS (MANUAL | X THEN Y) 🔥")
        self.get_logger().info("=========================================")

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------
    def telem_callback(self, msg: Telemetry):
        # Update arm state from actual telemetry
        self.is_armed = msg.arm

    def pose_callback(self, msg: PoseStamped):
        """Receives the exact center of the dock from the Perception Script"""
        self.last_pose_time = time.time()
        self.dock_visible = True

        self.target_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }

    # ---------------------------------------------------------
    # ROBUST STARTUP SEQUENCE
    # ---------------------------------------------------------
    def startup_sequence(self, now):
        """
        Safely sequences the Pixhawk: Disarm -> MANUAL -> Neutral -> Arm
        """
        if self.startup_state == 0:
            self.get_logger().info("Startup [1/4]: Disarming...")
            self.cmd_msg.arm = False
            self.cmd_msg.mode = "MANUAL"
            self.startup_time = now
            self.startup_state = 1

        elif self.startup_state == 1 and (now - self.startup_time) >= 1.0:
            self.get_logger().info("Startup [2/4]: Setting mode to MANUAL...")
            self.cmd_msg.mode = "MANUAL"
            self.cmd_msg.arm = False
            self.startup_time = now
            self.startup_state = 2

        elif self.startup_state == 2 and (now - self.startup_time) >= 1.0:
            self.get_logger().info("Startup [3/4]: Sending neutral thrust...")
            self.reset_commands()
            self.cmd_msg.mode = "MANUAL"
            self.cmd_msg.arm = False
            self.startup_time = now
            self.startup_state = 3

        elif self.startup_state == 3 and (now - self.startup_time) >= 1.0:
            self.get_logger().info("Startup [4/4]: Arming...")
            self.cmd_msg.mode = "MANUAL"
            self.cmd_msg.arm = True
            self.startup_time = now
            self.startup_state = 4
            
        elif self.startup_state == 4 and (now - self.startup_time) >= 1.0:
            self.get_logger().info("✅ STARTUP COMPLETE! Entering ASCEND phase.")
            self.startup_done = True
            self.state = "ASCEND"
            self.ascend_timer_start = now

        self.cmd_pub.publish(self.cmd_msg)

    # ---------------------------------------------------------
    # MAIN CONTROL LOOP
    # ---------------------------------------------------------
    def control_loop(self):
        now = time.time()
        
        # 1. Execute robust startup sequence first
        if not self.startup_done:
            self.startup_sequence(now)
            return

        # Cap dt to 100ms to prevent massive derivative spikes
        dt = min(now - self.prev_loop_time, 0.1)
        self.prev_loop_time = now

        # 2. Check if we lost the camera feed
        if now - self.last_pose_time > 1.0:
            if self.dock_visible:
                self.get_logger().error("❌ DOCK LOST! Marker out of frame!")
            self.dock_visible = False

        # 3. Ensure base commands are published
        self.cmd_msg.mode = "MANUAL" # Force MANUAL at all times
        self.cmd_msg.arm = True      # Force ARM at all times (unless DOCKED)
        self.reset_commands()        # Reset PWMs to 1500 every loop to prevent runaway

        # ==========================================
        # STATE MACHINE
        # ==========================================
        
        if self.state == "ASCEND":
            # Go UP for the first 2 seconds
            elapsed = now - self.ascend_timer_start
            
            self.cmd_msg.forward = 1500
            self.cmd_msg.lateral = 1500
            self.cmd_msg.yaw = 1500
            self.cmd_msg.thrust = self.ascend_pwm # e.g., 1600 (UP)
            
            self.log_status(self.state, f"GOING UP ({elapsed:.1f}/{self.ascend_duration}s)", 0, 0, 0)

            if elapsed >= self.ascend_duration:
                self.get_logger().info("✅ ASCEND COMPLETE! Moving to ALIGN_X.")
                self.pid_sway.reset()
                self.state = "ALIGN_X"

        # -------------------------------------------------
        # ALIGN X FIRST (Y is strictly neutral)
        # -------------------------------------------------
        elif self.state == "ALIGN_X":
            if not self.dock_visible:
                self.log_status("WAITING FOR DOCK...", "HOVER", 0, 0, 0)
            else:
                err_x = self.target_pose['x']
                err_y = self.target_pose['y']
                dist_z = self.target_pose['z']

                # Compute X PID, force Y to 0
                calc_sway  = int(self.pid_sway.compute(err_x, dt) * self.dir_sway)

                self.cmd_msg.lateral = self.pwm_neutral + calc_sway
                self.cmd_msg.forward = 1500 # <--- Y IS LOCKED NEUTRAL
                self.cmd_msg.thrust  = 1500 # Hover
                self.cmd_msg.yaw     = 1500 # Ignore Yaw

                self.log_status(self.state, "ALIGNING X (Y NEUTRAL)", err_x, err_y, dist_z)

                # Transition Condition: Only care if X is aligned
                if abs(err_x) < self.xy_tolerance:
                    self.get_logger().info("✅ X ALIGNED! Locking X and moving to ALIGN_Y.")
                    self.pid_surge.reset()
                    self.state = "ALIGN_Y"

        # -------------------------------------------------
        # ALIGN Y SECOND (X is actively locked via PID)
        # -------------------------------------------------
        elif self.state == "ALIGN_Y":
            if not self.dock_visible:
                self.pid_sway.reset()
                self.state = "ALIGN_X"
                return

            err_x = self.target_pose['x']
            err_y = self.target_pose['y']
            dist_z = self.target_pose['z']

            # Compute Y PID, actively hold X PID to lock it in place
            calc_sway  = int(self.pid_sway.compute(err_x, dt) * self.dir_sway)  # <-- X LOCKED
            calc_surge = int(self.pid_surge.compute(err_y, dt) * self.dir_surge) # <-- Y ALIGNING

            self.cmd_msg.lateral = self.pwm_neutral + calc_sway
            self.cmd_msg.forward = self.pwm_neutral + calc_surge
            self.cmd_msg.thrust  = 1500 # Hover
            self.cmd_msg.yaw     = 1500 # Ignore Yaw

            self.log_status(self.state, "ALIGNING Y (X LOCKED)", err_x, err_y, dist_z)

            # Drift Protection: If a current pushes X out of alignment, go back and fix it
            if abs(err_x) > self.xy_tolerance * 2.0:
                self.get_logger().warn("⚠️ X drifted off center! Reverting to ALIGN_X.")
                self.state = "ALIGN_X"
                
            # Transition Condition: Both X and Y are aligned
            elif abs(err_x) < self.xy_tolerance and abs(err_y) < self.xy_tolerance:
                self.get_logger().info("✅ X & Y ALIGNED! Starting DESCENT.")
                self.state = "DESCEND"

        # -------------------------------------------------
        # DESCEND (X & Y are actively locked)
        # -------------------------------------------------
        elif self.state == "DESCEND":
            if not self.dock_visible:
                # Reset PIDs so stale integrals don't cause a lurch when reacquired
                self.pid_sway.reset()
                self.pid_surge.reset()
                self.state = "ALIGN_X"
                return

            err_x  = self.target_pose['x']
            err_y  = self.target_pose['y']
            dist_z = self.target_pose['z']

            # Keep maintaining XY alignment while descending
            calc_sway  = int(self.pid_sway.compute(err_x, dt) * self.dir_sway)
            calc_surge = int(self.pid_surge.compute(err_y, dt) * self.dir_surge)

            self.cmd_msg.lateral = self.pwm_neutral + calc_sway
            self.cmd_msg.forward = self.pwm_neutral + calc_surge
            self.cmd_msg.yaw     = 1500 # Ignore Yaw
            
            # Push down (Value < 1500 sinks)
            if dist_z > 0.6:
                self.cmd_msg.thrust = 1450 # Slow descent
            else:
                self.cmd_msg.thrust = 1400 # Firm descent

            self.log_status(self.state, "SINKING (X&Y LOCKED)", err_x, err_y, dist_z)

            # Transition Condition: Enter the blind spot
            if dist_z < self.blind_spot_z:
                self.get_logger().info(f"⚠️ BLIND SPOT ENTERED (Z < {self.blind_spot_z}m). Starting BLIND LATCH.")
                self.blind_timer_start = now
                self.state = "BLIND_LATCH"

        # -------------------------------------------------
        # BLIND LATCH (Push down blindly)
        # -------------------------------------------------
        elif self.state == "BLIND_LATCH":
            self.cmd_msg.arm = True
            elapsed = now - self.blind_timer_start
            
            # Lock XY (neutral), thrust hard downwards to clamp the inductive pucks
            self.cmd_msg.forward = 1500
            self.cmd_msg.lateral = 1500
            self.cmd_msg.yaw = 1500
            self.cmd_msg.thrust = 1350 # Heavy downward pressure
            
            self.log_status(self.state, f"PUSHING DOWN ({elapsed:.1f}/{self.latch_duration}s)", 0, 0, 0)

            if elapsed > self.latch_duration:
                self.get_logger().info("🚀 DOCKING COMPLETE! MISSION ACCOMPLISHED.")
                self.state = "DOCKED"

        # -------------------------------------------------
        # DOCKED (Done)
        # -------------------------------------------------
        elif self.state == "DOCKED":
            # Disarm vehicle
            self.cmd_msg.arm = False
            self.reset_commands()
            self.get_logger().info("✅ DOCKED. System disarmed.", throttle_duration_sec=2.0)

        # 4. Publish the commands
        self.cmd_pub.publish(self.cmd_msg)

    # ---------------------------------------------------------
    # UTILITY FUNCTIONS
    # ---------------------------------------------------------
    def reset_commands(self):
        """Sets all thruster values to 1500 (Neutral)"""
        self.cmd_msg.forward = 1500
        self.cmd_msg.lateral = 1500
        self.cmd_msg.thrust = 1500
        self.cmd_msg.yaw = 1500
        self.cmd_msg.pitch = 1500
        self.cmd_msg.roll = 1500

    def log_status(self, phase, action, err_x, err_y, dist_z):
        """Rich debugging logger formatted nicely in the terminal"""
        
        # Only print this complex log every 0.5 seconds to avoid terminal spam
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = 0
            
        now = time.time()
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now

        dir_x_str = "RIGHT 👉" if (err_x * self.dir_sway) > 0 else "👈 LEFT " if (err_x * self.dir_sway) < 0 else "CENTER"
        dir_y_str = "BACK ⬇️" if (err_y * self.dir_surge) < 0 else "⬆️ FWD  " if (err_y * self.dir_surge) > 0 else "CENTER"
        
        log_str = (
            f"\n| PHASE: {phase:<12} | ACT: {action:<20} |\n"
            f"| X Err: {err_x:>6.2f}m ({dir_x_str:<8}) -> Lat PWM: {self.cmd_msg.lateral}\n"
            f"| Y Err: {err_y:>6.2f}m ({dir_y_str:<8}) -> Fwd PWM: {self.cmd_msg.forward}\n"
            f"| Z Dist:{dist_z:>6.2f}m             -> Thr PWM: {self.cmd_msg.thrust}\n"
            f"-------------------------------------------------"
        )
        self.get_logger().info(log_str)


def main(args=None):
    rclpy.init(args=args)
    node = Phase2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safe shutdown: Disarm
        node.get_logger().info("Shutting down... Disarming AUV.")
        cmd = Commands()
        cmd.arm = False
        cmd.mode = "MANUAL"
        cmd.forward = 1500; cmd.lateral = 1500; cmd.thrust = 1500; cmd.yaw = 1500
        node.cmd_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()