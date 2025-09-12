#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
from custom_msgs.msg import Commands, Telemetry, EmergencyKill
from std_srvs.srv import Empty
from rclpy.utilities import remove_ros_args
from optparse import OptionParser

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
        self.master_kill = True
        self.mode = auv_mode
        self.pixhawk_port = port_addr
        self.arm_state = False
        self.autonomy_switch = False

        # Initialize MAVLink connection
        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=115200)
        self.sys_status_msg = ardupilotmega.MAVLink_sys_status_message
        self.imu_msg = ardupilotmega.MAVLink_scaled_imu2_message
        self.attitude_msg = ardupilotmega.MAVLink_attitude_quaternion_message
        self.vfr_hud_msg = ardupilotmega.MAVLink_vfr_hud_message
        self.depth_msg = ardupilotmega.MAVLink_scaled_pressure2_message
        self.thruster_pwms_msg = ardupilotmega.MAVLink_servo_output_raw_message
        self.ahrs_msg = ardupilotmega.MAVLink_ahrs2_message

        # ROS 2 publisher and subscriber
        self.telemetry_pub = self.create_publisher(Telemetry, "/master/telemetry", 10)
        self.thruster_subs_rov = self.create_subscription(
            Commands, "/master/commands", self.rov_callback, 10
        )
        self.kill_sub = self.create_subscription(
            EmergencyKill, "/emergency_stop", self.kill_callback, 10
        )
        self.autonomy_service = self.create_service(
            Empty, "/mira/switch", self.service_callback
        )
        self.channel_ary = [1500] * 8  # Initialize channel values array
        self.master.wait_heartbeat()  # Wait for the heartbeat from the Pixhawk
        self.telem_msg = Telemetry()  # Initialize telemetry message

    def kill_callback(self, msg):
        if msg.kill_switch:
            self.disarm()
            self.get_logger().warn("KILL SWITCH ENABLED, DISARMING AND KILLING")
            exit()

    def service_callback(self, request, response):
        self.autonomy_switch = not self.autonomy_switch
        if self.autonomy_switch:
            self.get_logger().info("AUTONOMY MODE")
        else:
            self.get_logger().info("ROV MODE")
        return response

    def rov_callback(self, msg):
        # Handle arming/disarming
        if msg.arm == 1 and not self.arm_state:
            self.arm()
            self.arm_state = True
        elif msg.arm == 0 and self.arm_state:
            self.disarm()
            self.arm_state = False

        if not self.autonomy_switch:
            self.channel_ary[0] = msg.pitch
            self.channel_ary[1] = msg.roll
            self.channel_ary[2] = msg.thrust
            self.channel_ary[3] = msg.yaw
            self.channel_ary[4] = msg.forward
            self.channel_ary[5] = msg.lateral
            self.channel_ary[6] = msg.servo1
            self.channel_ary[7] = msg.servo2

            # Handle mode switching
            if self.mode != msg.mode:
                if not self.arm_state:
                    self.mode = msg.mode
                    self.mode_switch()
                else:
                    self.get_logger().warn("Disarm Pixhawk to change modes.")

    def arm(self):
        """
        Send an arm command to the Pixhawk.
        """
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
        """
        Send a disarm command to the Pixhawk.
        """
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
        """
        Switch the Pixhawk mode.
        """
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

    def set_rc_channel_pwm(self, id, pwm):
        """
        Set the PWM value for a specified RC channel.
        """
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
            )  # RC channel list, in microseconds

    def actuate(self):
        """
        Send RC channel commands to the Pixhawk based on updated channel values.
        """
        for i in range(8):
            self.set_rc_channel_pwm(i + 1, int(self.channel_ary[i]))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request the interval at which a specified message should be sent.
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,  # The MAVLink message ID
            1e6 / frequency_hz,  # The interval between two messages in microseconds
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

    def telem_publish_func(self, timestamp_now):
        """
        Publish telemetry data based on received MAVLink messages.
        """
        self.telem_msg.arm = self.arm_state
        self.telem_msg.battery_voltage = self.sys_status_msg.voltage_battery / 1000
        self.telem_msg.timestamp = timestamp_now
        
        self.telem_msg.internal_pressure = self.vfr_hud_msg.alt
        self.telem_msg.external_pressure = self.depth_msg.press_abs
        
        self.telem_msg.heading = self.vfr_hud_msg.heading
        
        self.telem_msg.imu_xacc = self.imu_msg.xacc
        self.telem_msg.imu_yacc = self.imu_msg.yacc
        self.telem_msg.imu_zacc = self.imu_msg.zacc

        self.telem_msg.imu_gyro_x = self.imu_msg.xgyro
        self.telem_msg.imu_gyro_y = self.imu_msg.ygyro
        self.telem_msg.imu_gyro_z = self.imu_msg.zgyro
        
        self.telem_msg.imu_gyro_compass_x = self.imu_msg.xmag
        self.telem_msg.imu_gyro_compass_y = self.imu_msg.ymag
        self.telem_msg.imu_gyro_compass_z = self.imu_msg.zmag
        
        self.telem_msg.q1 = self.attitude_msg.q1
        self.telem_msg.q2 = self.attitude_msg.q2
        self.telem_msg.q3 = self.attitude_msg.q3
        self.telem_msg.q4 = self.attitude_msg.q4
        self.telem_msg.rollspeed = self.attitude_msg.rollspeed
        self.telem_msg.pitchspeed = self.attitude_msg.pitchspeed
        self.telem_msg.yawspeed = self.attitude_msg.yawspeed

        self.telem_msg.roll = self.ahrs_msg.roll
        self.telem_msg.pitch = self.ahrs_msg.pitch
        self.telem_msg.yaw = self.ahrs_msg.yaw
        
        self.telem_msg.thruster_pwms[0] = self.thruster_pwms_msg.servo1_raw
        self.telem_msg.thruster_pwms[1] = self.thruster_pwms_msg.servo2_raw
        self.telem_msg.thruster_pwms[2] = self.thruster_pwms_msg.servo3_raw
        self.telem_msg.thruster_pwms[3] = self.thruster_pwms_msg.servo4_raw
        self.telem_msg.thruster_pwms[4] = self.thruster_pwms_msg.servo5_raw
        self.telem_msg.thruster_pwms[5] = self.thruster_pwms_msg.servo6_raw
        self.telem_msg.thruster_pwms[6] = self.thruster_pwms_msg.servo7_raw
        self.telem_msg.thruster_pwms[7] = self.thruster_pwms_msg.servo8_raw

        if self.telem_msg.battery_voltage < 15:
            # self.get_logger().warn(f"Battery Critically Low: {self.telem_msg.battery_voltage}V")
            pass
        self.telemetry_pub.publish(self.telem_msg)


def main(args=None):
    rclpy.init(args=args)

    # Command line options
    parser = OptionParser(description="description for prog")
    parser.add_option(
        "-p",
        "--port",
        dest="port_addr",
        default="/dev/Pixhawk",
        help="Pass Pixhawk Port Address",
        metavar="VAR",
    )
    parser.add_option(
        "-m",
        "--mode",
        dest="auv_mode",
        default="STABILIZE",
        help="Pass Pixhawk Mode",
        metavar="VAR",
    )
    (options, args) = parser.parse_args(args=remove_ros_args()[1:])

    # Instantiate class
    obj = PixhawkMaster(options.port_addr, options.auv_mode)

    # Request message intervals
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 100)

    try:
        # Receive MAVLink messages
        obj.sys_status_msg = obj.master.recv_match(type="SYS_STATUS", blocking=True)
        obj.imu_msg = obj.master.recv_match(type="SCALED_IMU2", blocking=True)
        obj.attitude_msg = obj.master.recv_match(type="ATTITUDE_QUATERNION", blocking=True)
        obj.vfr_hud_msg = obj.master.recv_match(type="VFR_HUD", blocking=True)
        obj.depth_msg = obj.master.recv_match(type="SCALED_PRESSURE2", blocking=True)
        obj.thruster_pwms_msg = obj.master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
        obj.ahrs_msg = obj.master.recv_match(type="AHRS2", blocking=True)
        obj.get_logger().info("All messages received once")
    except Exception as e:
        obj.get_logger().warn(f"Error receiving all messages: {e}")
        exit()

    # Main loop
    try:
        while rclpy.ok():
            obj.actuate()
            try:
                # Receive MAVLink messages
                obj.sys_status_msg = obj.master.recv_match(type="SYS_STATUS", blocking=True)
                obj.imu_msg = obj.master.recv_match(type="SCALED_IMU2", blocking=True)
                obj.attitude_msg = obj.master.recv_match(type="ATTITUDE_QUATERNION", blocking=True)
                obj.vfr_hud_msg = obj.master.recv_match(type="VFR_HUD", blocking=True)
                obj.depth_msg = obj.master.recv_match(type="SCALED_PRESSURE2", blocking=True)
                obj.thruster_pwms_msg = obj.master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
            except Exception as e:
                obj.get_logger().warn(f"Error receiving message: {e}")
                continue

            timestamp_now = obj.get_clock().now().to_msg().sec
            obj.telem_publish_func(timestamp_now)

            rclpy.spin_once(obj, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        obj.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
