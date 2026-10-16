"""Publish the existing custom telemetry contract from fresh MAVLink samples."""
import utils.units as uunits
from custom_msgs.msg import Telemetry
from pymavlink import mavutil
from utils import topics

from mira2_hardware.controller_plugin import ControllerPlugin
from mira2_hardware.mavlink_io import populate_scaled_imu


class TelemetryPublisher(ControllerPlugin):
    PIXHAWK_BODY_FRAME = 'base_link_frd'
    message_rates = {
        getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + name): rate
        for name, rate in (
            ('HEARTBEAT', 100.0), ('SYS_STATUS', 100.0),
            ('ATTITUDE_QUATERNION', 100.0), ('SCALED_PRESSURE2', 100.0),
            ('SCALED_PRESSURE', 100.0), ('VFR_HUD', 100.0),
            ('SERVO_OUTPUT_RAW', 100.0), ('SCALED_IMU2', 20.0), ('AHRS2', 100.0))
    }

    def __init__(self, controller):
        super().__init__(controller)
        self.last_telem = 0.0
        self.telemetry_pub = self.create_publisher(Telemetry, topics.TELEMETRY, 10)

    def tick(self, now):
        if now - self.last_telem >= 0.1:
            self.publish_telemetry()
            self.last_telem = now

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
            populate_scaled_imu(telemetry.imu, imu)
            telemetry.magnetic_field.magnetic_field.x = uunits.milligauss_to_tesla(imu.xmag)
            telemetry.magnetic_field.magnetic_field.y = uunits.milligauss_to_tesla(imu.ymag)
            telemetry.magnetic_field.magnetic_field.z = uunits.milligauss_to_tesla(imu.zmag)

        self.telemetry_pub.publish(telemetry)

