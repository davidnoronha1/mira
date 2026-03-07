#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Point
from std_msgs.msg import String
from custom_msgs.msg import Commands, Telemetry

class Phase:
    SEARCH = 0
    ALIGN_XY = 1
    APPROACH = 2
    LOCK = 3
    SEARCH2 = 4

PHASE_NAMES = {
    Phase.SEARCH: "SEARCHING",
    Phase.ALIGN_XY: "ALIGNING",
    Phase.APPROACH: "DIVING",
    Phase.LOCK: "LOCKED",
    Phase.SEARCH2: "EVADING"
}

class BucketControls(Node):

    def __init__(self):
        super().__init__("bucket_control_node")
        # PD Tuning
        self.kp_sway, self.kd_sway = 300.0, 100.0
        self.kp_surge, self.kd_surge = 300.0, 100.0
        self.kp_yaw = 5.0

        self.pwm_neutral = 1500
        self.pwm_max_effort = 400
        self.search_speed = 1600

        self.current_phase = Phase.SEARCH
        self.bucket_visible = False
        self.sees_blue = False
        self.sees_orange = False
        
        self.target_nx = 0.0
        self.target_ny = 0.0
        self.target_depth_proxy = 0.0
        self.last_time_seen = 0.0
        
        self.current_heading = 0.0
        self.locked_target_heading = None 
        
        self.prev_nx = 0.0
        self.prev_ny = 0.0
        self.last_brain_tick = time.time()
        
        self.blind_lock_timer = None
        self.evasion_sweep_start = None
        self.evasion_direction = 1

        self.cmd_pub = self.create_publisher(Commands, "/master/commands", 10)
        self.debug_pub = self.create_publisher(String, "bucket_debug", 10)


        self.create_subscription(Point, "bucket_target_2d", self.camera_callback, 10)
        self.create_subscription(Telemetry, "/master/telemetry", self.compass_callback, 10)
        self.create_subscription(String, "bucket_clr", self.color_callback, 10)

        self.create_timer(0.05, self.think_and_act)

    def compass_callback(self, msg):
        self.current_heading = msg.heading

    def camera_callback(self, msg):
        self.last_time_seen = time.time()
        self.bucket_visible = True
        self.target_nx = msg.x
        self.target_ny = msg.y
        self.target_depth_proxy = msg.z

    def color_callback(self, msg):
        self.sees_blue = (msg.data == "blue")
        self.sees_orange = (msg.data == "orange")

    def change_phase(self, new_phase, reason=""):
        if self.current_phase != new_phase:
            old_name = PHASE_NAMES.get(self.current_phase, "UNKNOWN")
            new_name = PHASE_NAMES.get(new_phase, "UNKNOWN")
            self.get_logger().warn(f"\n>>> STATE CHANGE: {old_name} -> {new_name} | {reason} <<<\n")
            self.current_phase = new_phase

    def think_and_act(self):
        cmd = Commands()
        cmd.mode = "ALT_HOLD"
        cmd.arm = 1
        cmd.pitch = cmd.roll = cmd.thrust = cmd.yaw = cmd.forward = cmd.lateral = self.pwm_neutral

        current_time = time.time()
        dt = current_time - self.last_brain_tick
        self.last_brain_tick = current_time

        if self.bucket_visible and (current_time - self.last_time_seen > 1.0):
            self.get_logger().warn("Target LOST from camera view! (1.0s timeout)")
            self.bucket_visible = self.sees_blue = self.sees_orange = False

        nx = self.target_nx if self.bucket_visible else 0.0
        ny = self.target_ny if self.bucket_visible else 0.0
        depth = self.target_depth_proxy if self.bucket_visible else 0.0

        if self.current_phase == Phase.SEARCH:
            cmd.forward = self.search_speed

            if self.bucket_visible and self.sees_blue:
                self.locked_target_heading = self.current_heading
                self.change_phase(Phase.ALIGN_XY, "Found BLUE target")

            elif self.bucket_visible and self.sees_orange:
                self.evasion_sweep_start = current_time
                self.evasion_direction = 1
                self.change_phase(Phase.SEARCH2, "Evading ORANGE target")

        elif self.current_phase == Phase.ALIGN_XY:
            if not self.bucket_visible:
                self.change_phase(Phase.SEARCH, "Target lost during alignment")
                return

            cmd.lateral = self.pwm_neutral + self.calculate_pd(nx, self.prev_nx, dt, self.kp_sway, self.kd_sway)
            cmd.forward = self.pwm_neutral - self.calculate_pd(ny, self.prev_ny, dt, self.kp_surge, self.kd_surge)
            cmd.yaw = self.calculate_heading_lock()

            if abs(nx) < 0.15 and abs(ny) < 0.15:
                self.change_phase(Phase.APPROACH, "XY centered. Diving.")

        elif self.current_phase == Phase.APPROACH:
            if self.bucket_visible and depth > 0.75:  
                self.blind_lock_timer = current_time
                self.change_phase(Phase.LOCK, "Target depth reached.")
                return

            if not self.bucket_visible and depth > 0.4:
                self.blind_lock_timer = current_time
                self.change_phase(Phase.LOCK, "Close range blind lock.")
                return
                
            elif not self.bucket_visible:
                self.change_phase(Phase.SEARCH, "Target lost high up. Aborting.")
                return

            cmd.lateral = self.pwm_neutral + self.calculate_pd(nx, self.prev_nx, dt, self.kp_sway, self.kd_sway)
            cmd.forward = self.pwm_neutral - self.calculate_pd(ny, self.prev_ny, dt, self.kp_surge, self.kd_surge)
            cmd.yaw = self.calculate_heading_lock()
            
            cmd.thrust = 1460 if depth > 0.4 else 1430

        elif self.current_phase == Phase.LOCK:
            if current_time - self.blind_lock_timer > 3.0:
                self.get_logger().info("Target Locked. Ready for drop.", throttle_duration_sec=2.0)

        elif self.current_phase == Phase.SEARCH2:
            if self.bucket_visible and self.sees_blue:
                self.change_phase(Phase.ALIGN_XY, "Found BLUE target")
                return
            
            cmd.lateral = self.pwm_neutral + (self.evasion_direction * 50)
            if (current_time - self.evasion_sweep_start) > 3.0:
                self.evasion_direction *= -1
                self.evasion_sweep_start = current_time
        self.prev_nx, self.prev_ny = nx, ny
        # Output
        self.log_motion(cmd)
        self.cmd_pub.publish(cmd)
        self.publish_debug(cmd, nx, ny, depth)


    # math & logic

    def calculate_pd(self, error, prev_error, dt, kp, kd):
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        output = int((error * kp) + (derivative * kd))
        return max(min(output, self.pwm_max_effort), -self.pwm_max_effort)

    def calculate_heading_lock(self):
        if self.locked_target_heading is None: return self.pwm_neutral
            
        err = self.locked_target_heading - self.current_heading
        if err > 180: err -= 360
        if err < -180: err += 360
        
        output = int(err * self.kp_yaw)
        return self.pwm_neutral + max(min(output, self.pwm_max_effort), -self.pwm_max_effort)

    # debugging thingies run topic echo bucket_debug

    def log_motion(self, cmd):

        if cmd.lateral > 1530:
            self.get_logger().info(f"-> Moving RIGHT (PWM: {cmd.lateral})", throttle_duration_sec=0.5)
        elif cmd.lateral < 1470:
            self.get_logger().info(f"<- Moving LEFT  (PWM: {cmd.lateral})", throttle_duration_sec=0.5)

        if cmd.forward > 1530:
            self.get_logger().info(f"^ Moving FWD    (PWM: {cmd.forward})", throttle_duration_sec=0.5)
        elif cmd.forward < 1470:
            self.get_logger().info(f"v Moving BACK   (PWM: {cmd.forward})", throttle_duration_sec=0.5)

        if cmd.thrust < 1480:
            self.get_logger().info(f"v DIVING DOWN   (PWM: {cmd.thrust})", throttle_duration_sec=0.5)

    def publish_debug(self, cmd, nx, ny, depth):
        state_str = PHASE_NAMES[self.current_phase]
        vis = "YES" if self.bucket_visible else "NO"
        color = "BLUE" if self.sees_blue else ("ORANGE" if self.sees_orange else "NONE")
        
        log_msg = (
            f"[{state_str:<9}] Vis:{vis}({color}) | "
            f"Err(nX,nY): {nx:>5.2f}, {ny:>5.2f} | DepthPx: {depth:>4.2f} | "
            f"PWM(F,L,Z,Y): {cmd.forward}, {cmd.lateral}, {cmd.thrust}, {cmd.yaw}"
        )
        self.debug_pub.publish(String(data=log_msg))
        self.get_logger().info(log_msg, throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = BucketControls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()