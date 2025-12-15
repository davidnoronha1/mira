#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
from custom_msgs.msg import Commands, Telemetry, EmergencyKill, Depth, Heading
import time

# Constants for channel mappings
# 1   Pitch
# 2   Roll
# 3   Throttle
# 4   Yaw
# 5   Forward
# 6   Lateral


class PixhawkMaster(Node):
    """
    A class to interface with a Pixhawk via MAVLink, handle mode switching,
    arming/disarming, and send telemetry data.
    """

    def __init__(self, port_addr, auv_mode):
        super().__init__("pymav_master")

        # When True, arming is inhibited until manually cleared
        self.master_kill = False
        self.emergency_locked = False
        self.mode = auv_mode
        self.pixhawk_port = port_addr
        self.arm_state = False
        self.autonomy_switch = False
        self.command_pwms = [1500] * 8  # Initialize channel values array

        # Initialize MAVLink connection and Mavlink msgs
        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=115200)
        self.msg_sys_status = ardupilotmega.MAVLink_sys_status_message
        self.msg_imu = ardupilotmega.MAVLink_scaled_imu2_message
        self.msg_attitude = ardupilotmega.MAVLink_attitude_quaternion_message
        self.msg_vfr_hud = ardupilotmega.MAVLink_vfr_hud_message
        self.msg_depth = ardupilotmega.MAVLink_scaled_pressure2_message
        self.pix_telemetry_thruster_pwms = ardupilotmega.MAVLink_servo_output_raw_message

        # ROS2 messages
        self.master_telem_msg = Telemetry()
        self.depth_msg = Depth()
        self.heading_msg = Heading()

        # ROS2 subscribers and publishers
        self.thruster_subs_rov = self.create_subscription(
            Commands, "/rov/commands", self.rov_callback, 10
        )
        # Subscribe to emergency topics (support either topic name)
        self.kill_sub = self.create_subscription(
            EmergencyKill, "/esp/telemetry", self.kill_callback, 10
        )
        self.kill_sub2 = self.create_subscription(
            EmergencyKill, "/emergency_stop", self.kill_callback, 10
        )
        # Service to clear emergency lock (manual reset)
        from std_srvs.srv import Empty
        self.clear_kill_srv = self.create_service(Empty, "/clear_emergency", self.clear_emergency)
        self.master_telemetry_pub = self.create_publisher(Telemetry, "/master/telemetry", 10)
        self.master_depth_pub = self.create_publisher(Depth, "/master/depth", 10)
        self.master_heading_pub = self.create_publisher(Heading, "/master/heading", 10)

        self.master.wait_heartbeat()  # Wait for the heartbeat from the Pixhawk

    # Callback Functions
    def kill_callback(self, msg):
        if msg.kill_switch:
            # Engage emergency lock and disarm immediately. Keep node alive
            # so it can continue to block further arming attempts.
            self.emergency_locked = True
            if self.arm_state:
                self.disarm()
                self.arm_state = False
            self.get_logger().warn("KILL SWITCH ENABLED: emergency lock engaged, disarming")
        else:
            # Clear emergency lock when switch is attached again (level pulled to GND)
            if self.emergency_locked:
                self.emergency_locked = False
                self.get_logger().info("Kill switch cleared: emergency lock released")

    def rov_callback(self, msg):
        if msg.arm == 1 and not self.arm_state:
            if self.emergency_locked:
                self.get_logger().warn("Arm blocked: emergency lock is engaged")
            else:
                self.arm()
                self.arm_state = True
        elif msg.arm == 0 and self.arm_state:
            self.disarm()
            self.arm_state = False

        if not self.autonomy_switch:
            self.command_pwms[0] = msg.pitch
            self.command_pwms[1] = msg.roll
            self.command_pwms[2] = msg.thrust
            self.command_pwms[3] = msg.yaw
            self.command_pwms[4] = msg.forward
            self.command_pwms[5] = msg.lateral
            self.command_pwms[6] = msg.servo1
            self.command_pwms[7] = msg.servo2

            # Handle mode switching
            if self.mode != msg.mode:
                if not self.arm_state:
                    self.mode = msg.mode
                    self.mode_switch()
                else:
                    self.get_logger().warn("Disarm Pixhawk to change modes.")

    # Publish Functions
    def master_telem_publish_func(self, timestamp_passed):
        self.master_telem_msg.battery_voltage = self.msg_sys_status.voltage_battery / 1000
        self.master_telem_msg.timestamp = timestamp_passed
        self.master_telem_msg.internal_pressure = self.msg_vfr_hud.alt
        self.master_telem_msg.external_pressure = self.msg_depth.press_abs
        self.master_telem_msg.heading = self.msg_vfr_hud.heading
        self.master_telem_msg.imu_gyro_x = self.msg_imu.xgyro
        self.master_telem_msg.imu_gyro_y = self.msg_imu.ygyro
        self.master_telem_msg.imu_gyro_z = self.msg_imu.zgyro
        self.master_telem_msg.imu_gyro_compass_x = self.msg_imu.xmag
        self.master_telem_msg.imu_gyro_compass_y = self.msg_imu.ymag
        self.master_telem_msg.q1 = self.msg_attitude.q1
        self.master_telem_msg.q2 = self.msg_attitude.q2
        self.master_telem_msg.q3 = self.msg_attitude.q3
        self.master_telem_msg.q4 = self.msg_attitude.q4
        self.master_telem_msg.rollspeed = self.msg_attitude.rollspeed
        self.master_telem_msg.pitchspeed = self.msg_attitude.pitchspeed
        self.master_telem_msg.yawspeed = self.msg_attitude.yawspeed
        self.master_telem_msg.thruster_pwms[0] = self.pix_telemetry_thruster_pwms.servo1_raw
        self.master_telem_msg.thruster_pwms[1] = self.pix_telemetry_thruster_pwms.servo2_raw
        self.master_telem_msg.thruster_pwms[2] = self.pix_telemetry_thruster_pwms.servo3_raw
        self.master_telem_msg.thruster_pwms[3] = self.pix_telemetry_thruster_pwms.servo4_raw
        self.master_telem_msg.thruster_pwms[4] = self.pix_telemetry_thruster_pwms.servo5_raw
        self.master_telem_msg.thruster_pwms[5] = self.pix_telemetry_thruster_pwms.servo6_raw
        self.master_telem_msg.thruster_pwms[6] = self.pix_telemetry_thruster_pwms.servo7_raw
        self.master_telem_msg.thruster_pwms[7] = self.pix_telemetry_thruster_pwms.servo8_raw

        if self.master_telem_msg.battery_voltage < 15:
            pass  # Handle low battery warning if needed

        self.master_telemetry_pub.publish(self.master_telem_msg)

    def depth_publish_func(self, timestamp_passed):
        self.depth_msg.external_pressure = self.msg_depth.press_abs
        self.depth_msg.timestamp = timestamp_passed
        self.master_depth_pub.publish(self.depth_msg)

    def heading_publish_func(self, timestamp_passed):
        self.heading_msg.heading = self.msg_vfr_hud.heading
        self.heading_msg.timestamp = timestamp_passed
        self.master_heading_pub.publish(self.heading_msg)

    # Pymavlink Functions
    def arm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # Arm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.get_logger().info("Arm command sent to Pixhawk")

    def disarm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # Disarm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.get_logger().info("Disarm command sent to Pixhawk")

    def mode_switch(self):
        if self.mode not in self.master.mode_mapping():
            self.get_logger().error(f"Unknown mode: {self.mode}")
            self.get_logger().info(f"Try: {list(self.master.mode_mapping().keys())}")
            exit(1)

        mode_id = self.master.mode_mapping()[self.mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.get_logger().info(f"Mode changed to: {self.mode}")

    def clear_emergency(self, request, response):
        """Service handler to clear an emergency lock."""
        if self.emergency_locked:
            self.emergency_locked = False
            self.get_logger().info("Emergency lock cleared via /clear_emergency service")
        else:
            self.get_logger().info("/clear_emergency called but no emergency was active")
        return response

    def set_rc_channel_pwm(self, id, pwm):
        if id < 1:
            self.get_logger().warn("Channel does not exist.")
            return

        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values,
            )

    def actuate(self):
        self.set_rc_channel_pwm(1, int(self.command_pwms[0]))
        self.set_rc_channel_pwm(2, int(self.command_pwms[1]))
        self.set_rc_channel_pwm(3, int(self.command_pwms[2]))
        self.set_rc_channel_pwm(4, int(self.command_pwms[3]))
        self.set_rc_channel_pwm(5, int(self.command_pwms[4]))
        self.set_rc_channel_pwm(6, int(self.command_pwms[5]))
        self.set_rc_channel_pwm(7, int(self.command_pwms[6]))
        self.set_rc_channel_pwm(8, int(self.command_pwms[7]))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            1e6 / frequency_hz,
            0,
            0,
            0,
            0,
            0,
        )
        response = self.master.recv_match(type="COMMAND_ACK", blocking=True)
        if (
            response
            and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        ):
            self.get_logger().info("Command Accepted")
        else:
            self.get_logger().error("Command Failed")


def main(args=None):
    rclpy.init(args=args)

    # Command line options
    import argparse

    parser = argparse.ArgumentParser(description="description for prog")
    parser.add_argument(
        "-p",
        "--port",
        dest="port_addr",
        default="/dev/Pixhawk",
        help="Pass Pixhawk Port Address",
    )
    parser.add_argument(
        "-m",
        "--mode",
        dest="auv_mode",
        default="STABILIZE",
        help="Pass Pixhawk Mode",
    )
    args = parser.parse_args(remove_ros_args()[1:])

    # Instantiate class
    pixhawk_master = PixhawkMaster(args.port_addr, args.auv_mode)

    # Request message intervals
    pixhawk_master.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 30)
    pixhawk_master.request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 30
    )
    pixhawk_master.request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 40
    )
    pixhawk_master.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 30)
    pixhawk_master.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 30)
    pixhawk_master.request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 30
    )

    try:
        # Receive MAVLink messages
        pixhawk_master.msg_sys_status = pixhawk_master.master.recv_match(
            type="SYS_STATUS", blocking=True
        )
        pixhawk_master.msg_imu = pixhawk_master.master.recv_match(
            type="SCALED_IMU2", blocking=True
        )
        pixhawk_master.msg_attitude = pixhawk_master.master.recv_match(
            type="ATTITUDE_QUATERNION", blocking=True
        )
        pixhawk_master.msg_vfr_hud = pixhawk_master.master.recv_match(
            type="VFR_HUD", blocking=True
        )
        pixhawk_master.msg_depth = pixhawk_master.master.recv_match(
            type="SCALED_PRESSURE2", blocking=True
        )
        pixhawk_master.msg_depth = pixhawk_master.master.recv_match(
            type="SERVO_OUTPUT_RAW", blocking=True
        )
        pixhawk_master.get_logger().info("All messages received once")

    except Exception as e:
        pixhawk_master.get_logger().warn(f"Error receiving all messages: {e}")
        exit()

    # Main loop
    try:
        while rclpy.ok():
            pixhawk_master.actuate()

            try:
                msg = pixhawk_master.master.recv_match(
                    type=[
                        "ATTITUDE_QUATERNION",
                        "SCALED_PRESSURE2",
                        "VFR_HUD",
                        "SCALED_IMU2",
                        "SERVO_OUTPUT_RAW",
                    ],
                    blocking=True,
                )
                timestamp_now = pixhawk_master.get_clock().now().to_msg().sec

                if not msg:
                    continue
                if msg.get_type() == "SYS_STATUS":
                    pixhawk_master.msg_sys_status = msg
                    pixhawk_master.master_telem_publish_func(timestamp_now)

                elif msg.get_type() == "SCALED_IMU2":
                    pixhawk_master.msg_imu = msg
                    pixhawk_master.master_telem_publish_func(timestamp_now)

                elif msg.get_type() == "ATTITUDE_QUATERNION":
                    pixhawk_master.msg_attitude = msg
                    pixhawk_master.master_telem_publish_func(timestamp_now)

                elif msg.get_type() == "VFR_HUD":
                    pixhawk_master.msg_vfr_hud = msg
                    pixhawk_master.master_telem_publish_func(timestamp_now)
                    pixhawk_master.heading_publish_func(timestamp_now)

                elif msg.get_type() == "SCALED_PRESSURE2":
                    pixhawk_master.msg_depth = msg
                    pixhawk_master.master_telem_publish_func(timestamp_now)
                    pixhawk_master.depth_publish_func(timestamp_now)

                elif msg.get_type() == "SERVO_OUTPUT_RAW":
                    pixhawk_master.pix_telemetry_thruster_pwms = msg
                    pixhawk_master.master_telem_publish_func(timestamp_now)

            except Exception as e:
                pixhawk_master.get_logger().warn(f"Error receiving message: {e}")
                continue

    except KeyboardInterrupt:
        pixhawk_master.get_logger().info("Shutting down PixhawkMaster node.")
    finally:
        pixhawk_master.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
