#!/usr/bin/env python3
"""Shared vehicle behavior for MAVLink IO implementations."""

import math

import rclpy
import utils.time as utime
import utils.units as uunits
from custom_msgs.msg import CommandsNormalized, Telemetry
from custom_msgs.srv import ChangeMode
from pymavlink import mavutil
from rclpy.node import Node
from std_srvs.srv import SetBool
from utils.control import COMMAND_FIELDS

from mira2_hardware.config import load_master_config


class MasterCore(Node):
    # Fixed runtime behavior.
    CONTROL_RATE_HZ = 50.0
    COMMAND_TIMEOUT_SECONDS = 0.5
    HEARTBEAT_TIMEOUT_SECONDS = 3.0
    TELEMETRY_TIMEOUT_SECONDS = 0.5
    TELEMETRY_RATE_HZ = 10.0
    IMU_RATE_HZ = 20.0
    MESSAGE_RATE_HZ = 100.0
    HEARTBEAT_RATE_HZ = 100.0
    RECEIVE_BUDGET = 64
    COMMAND_INPUT_TOPIC = "command"
    RC_CHANNEL_FIELDS = ("pitch", "roll", "thrust", "yaw", "forward", "lateral")
    UNUSED_RC_CHANNEL = 65535
    PIXHAWK_BODY_FRAME = "base_link_frd"

    def __init__(self, connection=None, node_name="master"):
        super().__init__(node_name)

        # Configuration and MAVLink connection.
        self.calibration = load_master_config(self)
        self.master = connection or mavutil.mavlink_connection(
            self.get_parameter("pixhawk_address").value,
            baud=self.get_parameter("baudrate").value)
        self.mode = self.get_parameter("initial_mode").value

        # Runtime state.
        self.ready = False
        self.arm_state = False
        self.requested_arm = False
        self.emergency_active = False
        self.last_command = None
        self.last_heartbeat = None
        self.last_telem = 0.0
        self.message_intervals_configured = False
        self.update_channels()
        self.messages = {}
        self.message_times = {}

        # ROS interfaces.
        self.telemetry_pub = self.create_publisher(Telemetry, "telemetry", 10)
        self.create_subscription(
            CommandsNormalized,
            self.COMMAND_INPUT_TOPIC,
            self.command_callback,
            1,
        )
        self.create_service(SetBool, "emergency_stop", self.set_emergency_stop)
        self.create_service(SetBool, "arm", self.set_arm)
        self.create_service(ChangeMode, "change_mode", self.set_mode)
        self.create_timer(utime.seconds_from_rate(self.CONTROL_RATE_HZ), self.tick)

    def update_channels(self, values=None):
        """Store calibrated RC channels, or neutral channels when values are absent."""
        if values is None or values == 0:
            channels = [self.calibration[field].neutral for field in self.RC_CHANNEL_FIELDS]
        else:
            channels = [self.calibration[field].to_pwm(values[field])
                        for field in self.RC_CHANNEL_FIELDS]
        self.channels = channels + [self.UNUSED_RC_CHANNEL, self.UNUSED_RC_CHANNEL]

    def command_callback(self, msg: CommandsNormalized):
        if not self.ready or self.emergency_active:
            return
        if not utime.is_fresh(
                utime.stamp_age(self.get_clock(), msg.header.stamp),
                self.COMMAND_TIMEOUT_SECONDS):
            self.stop("Stale or future-stamped command")
            return
        if msg.header.frame_id != "base_link":
            self.stop("Effort command must use the base_link FLU frame")
            return
        try:
            values = {field: float(getattr(msg, field)) for field in COMMAND_FIELDS}
            if any(not math.isfinite(value) or abs(value) > 1 for value in values.values()):
                raise ValueError("Effort outside [-1, 1]")
        except ValueError:
            self.stop("Nonfinite or out-of-range command")
            return
        if msg.arm and not self.requested_arm and self.mode == "GUIDED":
            self.stop("Arm in MANUAL/STABILIZE before selecting GUIDED")
            return
        self.update_fcu(armed=bool(msg.arm))
        if msg.mode and msg.mode != self.mode:
            if self.requested_arm:
                self.get_logger().warn("Disarm Pixhawk to change modes.")
            elif not self.update_fcu(mode=msg.mode):
                self.get_logger().error(f"Unknown mode: {msg.mode}")
        if not msg.arm:
            self.update_channels()
        else:
            self.update_channels(values)
        self.last_command = utime.monotonic_now()

    def update_fcu(self, *, armed=None, mode=None, force_mode=False):
        """Request an arm-state or mode change from the flight controller."""
        if armed is not None and (armed != self.requested_arm or armed != self.arm_state):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                int(armed),
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.requested_arm = armed

        if mode and (force_mode or mode != self.mode):
            mapping = self.master.mode_mapping() or {}
            if mode not in mapping:
                return False
            self.master.mav.set_mode_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode],
            )
            self.mode = mode

        return True

    def stop(self, reason):
        self.update_channels()
        self.last_command = None
        self.update_fcu(armed=False)
        self.get_logger().warn(reason)

    def set_emergency_stop(self, request, response):
        self.emergency_active = request.data
        self.stop("Emergency engaged" if request.data else "Emergency cleared; remain disarmed")
        response.success = True
        response.message = "Emergency engaged" if request.data else "Emergency cleared"
        return response

    def set_arm(self, request, response):
        if not self.ready:
            response.success = False
            response.message = "Autopilot not ready"
        elif request.data and self.emergency_active:
            response.success = False
            response.message = "Emergency stop is active"
        elif request.data and self.mode == "GUIDED":
            response.success = False
            response.message = "Select MANUAL/STABILIZE before arming"
        else:
            self.update_channels()
            self.update_fcu(armed=request.data)
            response.success = True
            response.message = "Arm requested" if request.data else "Disarm requested"
        return response

    def set_mode(self, request, response):
        mode = request.mode.strip().upper()
        if not self.ready:
            response.success = False
            response.message = "Autopilot not ready"
        elif self.emergency_active:
            response.success = False
            response.message = "Emergency is active"
        elif self.arm_state or self.requested_arm:
            response.success = False
            response.message = "Disarm before changing modes"
        elif not mode:
            response.success = False
            response.message = "Mode is required"
        elif not self.update_fcu(mode=mode):
            response.success = False
            response.message = f"Unknown mode: {mode}"
        else:
            response.success = True
            response.message = f"Mode changed to {mode}"
        return response

    def receive_messages(self):
        raise NotImplementedError

    def tick(self):
        for msg in self.receive_messages():
            now = utime.monotonic_now()
            kind = msg.get_type()
            if kind == "HEARTBEAT" and getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue  # Ground-station heartbeats must not mark the FCU ready.
            self.messages[kind] = msg
            self.message_times[kind] = now
            if kind == "HEARTBEAT":
                self.last_heartbeat = now
                self.arm_state = bool(getattr(msg, "base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not self.ready:
                    self.ready = True
                    if self.arm_state:
                        self.stop("Autopilot connected while armed; requesting disarm")
                    else:
                        self.update_fcu(mode=self.mode, force_mode=True)
                    self.configure_message_intervals()
            if kind == "COMMAND_ACK" \
                and msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL \
                and msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().warn(f"Autopilot rejected a stream interval request: result={msg.result}")
        now = utime.monotonic_now()
        if not self.ready:
            return
        if utime.elapsed(self.last_heartbeat, now) > self.HEARTBEAT_TIMEOUT_SECONDS:
            self.stop("Autopilot heartbeat lost")
            self.ready = False
        elif (self.last_command is not None
              and utime.elapsed(self.last_command, now) > self.COMMAND_TIMEOUT_SECONDS):
            self.stop("Command input timed out")
        self.actuate()
        if utime.elapsed(self.last_telem, now) >= utime.seconds_from_rate(self.TELEMETRY_RATE_HZ):
            self.publish_telemetry()
            self.last_telem = now

    def actuate(self):
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component, *self.channels)

    def configure_message_intervals(self):
        if self.message_intervals_configured:
            return

        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
            mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
        ):
            frequency = self.MESSAGE_RATE_HZ
            if message_id == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                frequency = self.HEARTBEAT_RATE_HZ
            elif message_id == mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2:
                frequency = self.IMU_RATE_HZ
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                1e6 / frequency,
                0,
                0,
                0,
                0,
                0,
            )

        self.message_intervals_configured = True

    def publish_telemetry(self):
        telemetry = Telemetry()
        telemetry.header.stamp = self.get_clock().now().to_msg()
        telemetry.header.frame_id = self.PIXHAWK_BODY_FRAME
        telemetry.armed.data = self.requested_arm
        telemetry.battery.voltage = float("nan")
        telemetry.external_pressure.fluid_pressure = float("nan")
        telemetry.internal_pressure.fluid_pressure = float("nan")
        telemetry.external_pressure.header = telemetry.header
        telemetry.internal_pressure.header = telemetry.header
        telemetry.heading_ned_degrees.data = float("nan")
        telemetry.imu.header = telemetry.header
        telemetry.imu.orientation_covariance[0] = -1.0
        telemetry.magnetic_field.header = telemetry.header

        status = self.fresh_message("SYS_STATUS")
        if status is not None:
            telemetry.battery.voltage = uunits.millivolts_to_volts(status.voltage_battery)

        hud = self.fresh_message("VFR_HUD")
        if hud is not None:
            telemetry.heading_ned_degrees.data = float(hud.heading)

        external_pressure = self.fresh_message("SCALED_PRESSURE2")
        if external_pressure is not None:
            telemetry.external_pressure.fluid_pressure = uunits.hectopascals_to_pascals(
                external_pressure.press_abs)

        internal_pressure = self.fresh_message("SCALED_PRESSURE")
        if internal_pressure is not None:
            telemetry.internal_pressure.fluid_pressure = uunits.hectopascals_to_pascals(
                internal_pressure.press_abs)

        attitude = self.fresh_message("AHRS2")
        if attitude is not None:
            telemetry.attitude_rpy_ned.x = attitude.roll
            telemetry.attitude_rpy_ned.y = attitude.pitch
            telemetry.attitude_rpy_ned.z = attitude.yaw

        orientation = self.fresh_message("ATTITUDE_QUATERNION")
        if orientation is not None:
            telemetry.imu.orientation.w = orientation.q1
            telemetry.imu.orientation.x = orientation.q2
            telemetry.imu.orientation.y = orientation.q3
            telemetry.imu.orientation.z = orientation.q4
            telemetry.imu.orientation_covariance[0] = 0.0

        raw = self.fresh_message("SERVO_OUTPUT_RAW")
        if raw is not None:
            telemetry.thruster_pwms.data = [float(getattr(raw, f"servo{i}_raw")) for i in range(1, 9)]

        imu = self.fresh_message("SCALED_IMU2")
        if imu is not None:
            telemetry.imu.linear_acceleration.x = uunits.milligravity_to_meters_per_second_squared(imu.xacc)
            telemetry.imu.linear_acceleration.y = uunits.milligravity_to_meters_per_second_squared(imu.yacc)
            telemetry.imu.linear_acceleration.z = uunits.milligravity_to_meters_per_second_squared(imu.zacc)
            telemetry.imu.angular_velocity.x = uunits.milliradians_per_second_to_radians_per_second(imu.xgyro)
            telemetry.imu.angular_velocity.y = uunits.milliradians_per_second_to_radians_per_second(imu.ygyro)
            telemetry.imu.angular_velocity.z = uunits.milliradians_per_second_to_radians_per_second(imu.zgyro)
            telemetry.magnetic_field.magnetic_field.x = uunits.milligauss_to_tesla(imu.xmag)
            telemetry.magnetic_field.magnetic_field.y = uunits.milligauss_to_tesla(imu.ymag)
            telemetry.magnetic_field.magnetic_field.z = uunits.milligauss_to_tesla(imu.zmag)

        self.telemetry_pub.publish(telemetry)

    def fresh_message(self, kind):
        if utime.elapsed(self.message_times.get(kind, 0.0)) <= self.TELEMETRY_TIMEOUT_SECONDS:
            return self.messages.get(kind)
        return None


def run(node_class, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop("Master shutdown")
            node.actuate()
            node.master.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
