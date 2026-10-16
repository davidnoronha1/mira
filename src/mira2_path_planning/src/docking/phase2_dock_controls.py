#!/usr/bin/env python3

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import CommandsNormalized, Telemetry
import time
from utils import topics
from utils.control import PIDController, publish_mission_command
from utils.parameters import dataclass_config, declare_config, positive


@dataclass(frozen=True)
class DockingConfig:
    """Local defaults for ROS parameter overrides."""

    control_rate: float = 20.0
    feedback_timeout: float = 1.0
    descent_far_effort: float = -0.125
    descent_near_effort: float = -0.25
    latch_effort: float = -0.375
    descent_threshold: float = 0.6
    dir_sway: int = 1
    dir_surge: int = -1
    latch_duration: float = 10.0
    ascend_duration: float = 2.0
    ascend_effort: float = 0.25
    blind_spot_z: float = 0.3
    xy_tolerance: float = 0.05


# ==========================================
# ROBUST PID CONTROLLER
# ==========================================
# ==========================================
# MAIN CONTROLLER NODE
# ==========================================
class Phase2Controller(Node):
    def __init__(self):
        super().__init__("phase2_controls")
        self.config = dataclass_config(self, DockingConfig(), validators={
            "control_rate": positive, "feedback_timeout": positive})

        # =========================================================
        # 2. TUNABLE PID PARAMETERS (No Yaw)
        # =========================================================
        self.pid_sway = PIDController(**declare_config(
            self, {"kp": 0.5, "ki": 0.0, "kd": 0.125}, namespace="sway"))
        self.pid_surge = PIDController(**declare_config(
            self, {"kp": 0.5, "ki": 0.0, "kd": 0.125}, namespace="surge"))

        # =========================================================
        # 3. CONFIGURATION
        # =========================================================
        self.effort_neutral = 0.0

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
        self.cmd_msg = CommandsNormalized()
        self.cmd_msg.mode = "MANUAL" # STRICTLY MANUAL
        self.cmd_msg.arm = False
        self.reset_commands()

        # --- Telemetry state ---
        self.is_armed = False

        # --- ROS Interfaces ---
        self.cmd_pub = self.create_publisher(CommandsNormalized, topics.COMMAND, 10)
        self.create_subscription(PoseStamped, topics.BOTTOM_DOCK_POSE, self.pose_callback, 10)
        self.create_subscription(Telemetry, topics.TELEMETRY, self.telem_callback, 10)

        # Run loop at 20Hz (0.05s)
        self.create_timer(1.0 / self.config.control_rate, self.control_loop)

        self.get_logger().info("=========================================")
        self.get_logger().info("🔥 PHASE 2 CONTROLS (MANUAL | X THEN Y) 🔥")
        self.get_logger().info("=========================================")

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------
    def telem_callback(self, msg: Telemetry):
        # Update arm state from actual telemetry
        self.is_armed = msg.armed.data

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

        publish_mission_command(self, self.cmd_pub, self.cmd_msg)

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
        if now - self.last_pose_time > self.config.feedback_timeout:
            if self.dock_visible:
                self.get_logger().error("❌ DOCK LOST! Marker out of frame!")
            self.dock_visible = False

        # 3. Ensure base commands are published
        self.cmd_msg.mode = "MANUAL" # Force MANUAL at all times
        self.cmd_msg.arm = True      # Force ARM at all times (unless DOCKED)
        self.reset_commands()        # Reset efforts to 0.0 every loop to prevent runaway

        # ==========================================
        # STATE MACHINE
        # ==========================================

        if self.state == "ASCEND":
            # Go UP for the first 2 seconds
            elapsed = now - self.ascend_timer_start

            self.cmd_msg.forward = 0.0
            self.cmd_msg.lateral = 0.0
            self.cmd_msg.yaw = 0.0
            self.cmd_msg.thrust = self.config.ascend_effort # e.g., 0.25 (UP)

            self.log_status(self.state, f"GOING UP ({elapsed:.1f}/{self.config.ascend_duration}s)", 0, 0, 0)

            if elapsed >= self.config.ascend_duration:
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
                calc_sway  = float(self.pid_sway.control(err_x, dt) * self.config.dir_sway)

                self.cmd_msg.lateral = self.effort_neutral + calc_sway
                self.cmd_msg.forward = 0.0 # <--- Y IS LOCKED NEUTRAL
                self.cmd_msg.thrust  = 0.0 # Hover
                self.cmd_msg.yaw     = 0.0 # Ignore Yaw

                self.log_status(self.state, "ALIGNING X (Y NEUTRAL)", err_x, err_y, dist_z)

                # Transition Condition: Only care if X is aligned
                if abs(err_x) < self.config.xy_tolerance:
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
            calc_sway  = float(self.pid_sway.control(err_x, dt) * self.config.dir_sway)  # <-- X LOCKED
            calc_surge = float(self.pid_surge.control(err_y, dt) * self.config.dir_surge) # <-- Y ALIGNING

            self.cmd_msg.lateral = self.effort_neutral + calc_sway
            self.cmd_msg.forward = self.effort_neutral + calc_surge
            self.cmd_msg.thrust  = 0.0 # Hover
            self.cmd_msg.yaw     = 0.0 # Ignore Yaw

            self.log_status(self.state, "ALIGNING Y (X LOCKED)", err_x, err_y, dist_z)

            # Drift Protection: If a current pushes X out of alignment, go back and fix it
            if abs(err_x) > self.config.xy_tolerance * 2.0:
                self.get_logger().warn("⚠️ X drifted off center! Reverting to ALIGN_X.")
                self.state = "ALIGN_X"

            # Transition Condition: Both X and Y are aligned
            elif abs(err_x) < self.config.xy_tolerance and abs(err_y) < self.config.xy_tolerance:
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
            calc_sway  = float(self.pid_sway.control(err_x, dt) * self.config.dir_sway)
            calc_surge = float(self.pid_surge.control(err_y, dt) * self.config.dir_surge)

            self.cmd_msg.lateral = self.effort_neutral + calc_sway
            self.cmd_msg.forward = self.effort_neutral + calc_surge
            self.cmd_msg.yaw     = 0.0 # Ignore Yaw

            # Push down (Value < 0.0 sinks)
            if dist_z > self.config.descent_threshold:
                self.cmd_msg.thrust = self.config.descent_far_effort # Slow descent
            else:
                self.cmd_msg.thrust = self.config.descent_near_effort # Firm descent

            self.log_status(self.state, "SINKING (X&Y LOCKED)", err_x, err_y, dist_z)

            # Transition Condition: Enter the blind spot
            if dist_z < self.config.blind_spot_z:
                self.get_logger().info(f"⚠️ BLIND SPOT ENTERED (Z < {self.config.blind_spot_z}m). Starting BLIND LATCH.")
                self.blind_timer_start = now
                self.state = "BLIND_LATCH"

        # -------------------------------------------------
        # BLIND LATCH (Push down blindly)
        # -------------------------------------------------
        elif self.state == "BLIND_LATCH":
            self.cmd_msg.arm = True
            elapsed = now - self.blind_timer_start

            # Lock XY (neutral), thrust hard downwards to clamp the inductive pucks
            self.cmd_msg.forward = 0.0
            self.cmd_msg.lateral = 0.0
            self.cmd_msg.yaw = 0.0
            self.cmd_msg.thrust = self.config.latch_effort # Heavy downward pressure

            self.log_status(self.state, f"PUSHING DOWN ({elapsed:.1f}/{self.config.latch_duration}s)", 0, 0, 0)

            if elapsed > self.config.latch_duration:
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
        publish_mission_command(self, self.cmd_pub, self.cmd_msg)

    # ---------------------------------------------------------
    # UTILITY FUNCTIONS
    # ---------------------------------------------------------
    def reset_commands(self):
        """Sets all thruster values to 0.0 (Neutral)"""
        self.cmd_msg.forward = 0.0
        self.cmd_msg.lateral = 0.0
        self.cmd_msg.thrust = 0.0
        self.cmd_msg.yaw = 0.0
        self.cmd_msg.pitch = 0.0
        self.cmd_msg.roll = 0.0

    def log_status(self, phase, action, err_x, err_y, dist_z):
        """Rich debugging logger formatted nicely in the terminal"""

        # Only print this complex log every 0.5 seconds to avoid terminal spam
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = 0

        now = time.time()
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now

        dir_x_str = "RIGHT 👉" if (err_x * self.config.dir_sway) > 0 else "👈 LEFT " if (err_x * self.config.dir_sway) < 0 else "CENTER"
        dir_y_str = "BACK ⬇️" if (err_y * self.config.dir_surge) < 0 else "⬆️ FWD  " if (err_y * self.config.dir_surge) > 0 else "CENTER"

        log_str = (
            f"\n| PHASE: {phase:<12} | ACT: {action:<20} |\n"
            f"| X Err: {err_x:>6.2f}m ({dir_x_str:<8}) -> Lat effort: {self.cmd_msg.lateral}\n"
            f"| Y Err: {err_y:>6.2f}m ({dir_y_str:<8}) -> Fwd effort: {self.cmd_msg.forward}\n"
            f"| Z Dist:{dist_z:>6.2f}m             -> Thr effort: {self.cmd_msg.thrust}\n"
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
        cmd = CommandsNormalized()
        cmd.arm = False
        cmd.mode = "MANUAL"
        cmd.forward = 0.0; cmd.lateral = 0.0; cmd.thrust = 0.0; cmd.yaw = 0.0
        publish_mission_command(node, node.cmd_pub, cmd)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
