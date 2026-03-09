# NOTE: This node contains both perception and controls logic.

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_msgs.msg import Telemetry
from custom_msgs.msg import Commands

from cv_bridge import CvBridge
import cv2
import time
import math
from ultralytics import YOLO
from ament_index_python import get_package_share_directory

class DepthYawVisionController(Node):

    def __init__(self):
        super().__init__('depth_yaw_vision_controller')

        self.cmd_pub = self.create_publisher(Commands, '/master/commands', 10)
        self.telemetry_sub = self.create_subscription(Telemetry, '/master/telemetry', self.telemetry_cb, 10)

        #replace with real auv camera topic
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)


        # -------------------- Vision --------------------
        self.bridge = CvBridge()
        model_path = os.path.join(
            get_package_share_directory("mira2_pid_control"),
            "resource",
            "buckets_p1_first_attempt.pt"
        )

        self.model = YOLO(model_path)
        self.get_logger().info(f"Loading model from: {model_path}")

        # -------------------- State --------------------
        self.current_depth = None
        self.current_yaw = None
        self.target_yaw = None
        self.target_depth = 1.3  # FIXED depth target

        # -------------------- Gains --------------------
        self.depth_Kp = 45.0
        self.depth_Kd = 25.0

        self.yaw_Kp = 40.0
        self.yaw_Kd = 0.0

        # -------------------- Motion intent --------------------
        self.surge = 0.0
        self.yaw_cmd = 0.0
        self.heave = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.sway = 0.0   # Always zero now


        # -------------------- PID State --------------------
        self.prev_depth_error = 0.0
        self.prev_yaw_error = 0.0
        self.prev_time = time.time()

        self.start_time = None

        self.get_logger().info("Depth + Yaw + Vision (Display Only) Controller Started")

        self.control_timer = self.create_timer(0.1, self.control_loop)

    # TELELMETRY
    def telemetry_cb(self,msg):

        # #IMPORTANT!!! check here regarding how the pressure values work! we might need to change this part later!
        # pressure = msg.external_pressure
        # surface_pressure = 1013.0      # replace with actual measured surface value
        # pressure_at_1_3m = 1075.0      # your known value of pressure at 1.3 metres
        # scale = 1.3 / (pressure_at_1_3m - surface_pressure)

        # self.current_depth = (pressure - surface_pressure) * scale


        #idk if this works. lets see..
        self.current_depth = msg.depth


        self.current_yaw = msg.yaw

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.start_time = time.time()
            self.get_logger().info(f"Yaw locked at {self.target_yaw:.3f} rad")


    # IMAGE CALLBACK (VISION DISPLAY ONLY)
    def image_cb(self, msg):
        if self.current_depth is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        frame_center_x = w // 2
        top_line = int(0.7 * h)
        bottom_line = int(0.8 * h)

        results = self.model(frame, conf=0.5, classes=[8])

        sway_text = "NONE"
        heave_text = "HOLD"

        k_heave = 0.001
        k_sway = 0.30

        # Reset sway every frame
        self.sway = 0.0

        for r in results:
            boxes = r.boxes
            if boxes is not None and len(boxes) > 0:
                best_box = max(boxes, key=lambda b: float(b.conf))
                x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()

                blue_bucket_center_x = int((x1 + x2) / 2)
                blue_bucket_center_y = int((y1 + y2) / 2)

                cv2.rectangle(frame,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            (0, 255, 0), 2)

                cv2.circle(frame,
                        (blue_bucket_center_x, blue_bucket_center_y),
                        5,
                        (0, 0, 255), -1)

                # ---------------------------------------------------
                # AFTER 17 SECONDS → ENABLE HEAVE + SWAY CONTROL
                # ---------------------------------------------------
                if self.start_time is not None:
                    elapsed = time.time() - self.start_time

                    if elapsed > 17.0:

                        # ---------------- HEAVE ----------------
                        if blue_bucket_center_y < top_line:
                            error_pixels = top_line - blue_bucket_center_y
                            self.target_depth -= k_heave * error_pixels
                            heave_text = "UP"

                        elif blue_bucket_center_y > bottom_line:
                            error_pixels = blue_bucket_center_y - bottom_line
                            self.target_depth += k_heave * error_pixels
                            heave_text = "DOWN"

                        else:
                            heave_text = "HOLD"
                        
                        # Clamp target depth to maximum 2.5 meters
                        if self.target_depth > 2.5:
                            self.target_depth = 2.5


                        # ---------------- SWAY (PROPORTIONAL) ----------------
                        offset_x = blue_bucket_center_x - frame_center_x

                        self.sway = -k_sway * offset_x

                        if offset_x <0:
                            sway_text = "LEFT"
                        elif offset_x > 0:
                            sway_text = "RIGHT"
                        else:
                            sway_text = "CENTERED"

                    else:
                        heave_text = "HOLD"
                        sway_text = "NONE"

                break

        # Guide lines
        cv2.line(frame, (frame_center_x, 0),
                (frame_center_x, h), (255, 0, 0), 2)

        cv2.line(frame, (0, top_line),
                (w, top_line), (255, 255, 0), 2)

        cv2.line(frame, (0, bottom_line),
                (w, bottom_line), (255, 255, 0), 2)

        cv2.putText(frame, f"sway: {sway_text}",
                    (40, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255), 2)

        cv2.putText(frame, f"heave: {heave_text}",
                    (40, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255), 2)

        cv2.imshow("Blue Bucket Alignment", frame)
        cv2.waitKey(1)


    # ==========================================================
    # CONTROL LOOP
    # ==========================================================

    def control_loop(self):

        if self.current_depth is None or self.current_yaw is None:
            return

        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        # ---------------- Depth PD ----------------
        depth_error = self.current_depth - self.target_depth
        depth_derivative = (
            (depth_error - self.prev_depth_error) / dt
            if dt > 0.0 else 0.0
        )
        self.prev_depth_error = depth_error

        depth_output = (
            self.depth_Kp * depth_error +
            self.depth_Kd * depth_derivative
        )

        # ---------------- Yaw PD ----------------
        yaw_error = self.angle_error(
            self.target_yaw,
            self.current_yaw
        )

        yaw_derivative = (
            (yaw_error - self.prev_yaw_error) / dt
            if dt > 0.0 else 0.0
        )
        self.prev_yaw_error = yaw_error

        yaw_output = (
            self.yaw_Kp * yaw_error +
            self.yaw_Kd * yaw_derivative
        )

        # ---------------- Surge logic ----------------
        elapsed = time.time() - self.start_time if self.start_time is not None else 0.0
        surge_command = 0.0 if elapsed < 17.0 else 5.0

        # ---------------- Assign ----------------
        self.surge = surge_command
        self.yaw_cmd = yaw_output
        self.heave = depth_output

        self.publish_commands()

    # ==========================================================

    def angle_error(self, target, current):
        e = target - current
        while e > math.pi:
            e -= 2.0 * math.pi
        while e < -math.pi:
            e += 2.0 * math.pi
        return e


    def publish_commands(self):
        cmd = Commands()
        cmd.arm = True
        cmd.mode = "ALT_HOLD"

        cmd.forward = int(1500+self.surge)
        cmd.lateral = int(1500 + self.sway)
        cmd.thrust  = int(1500 - self.heave)   # minus because depth sign usually inverted. BUT VERIFY AGAIN IF THIS IS CORRECT OR NOT!!
        cmd.yaw     = int(1500 + self.yaw_cmd)
        cmd.roll    = 1500
        
        self.cmd_pub.publish(cmd)



def main():
    rclpy.init()
    node = DepthYawVisionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    
