"""MAVLink transport operations and conversions for the Pixhawk controller."""

from pymavlink import mavutil
from utils import units


class MavlinkIO:
    """Protocol operations on ``master``; control policy belongs to the node."""

    RECEIVE_BUDGET = 64

    def receive_messages(self, message_types=None):
        for _ in range(self.RECEIVE_BUDGET):
            msg = self.master.recv_match(type=message_types, blocking=False)
            if msg is None:
                break
            yield msg

    def request_arm(self, armed):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            int(armed), 0, 0, 0, 0, 0, 0)

    def request_mode(self, custom_mode):
        self.master.mav.set_mode_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode)

    def request_message_interval(self, message_id, frequency):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 1e6 / frequency, 0, 0, 0, 0, 0)


def autopilot_heartbeat(msg):
    return getattr(msg, 'autopilot', mavutil.mavlink.MAV_AUTOPILOT_INVALID) != mavutil.mavlink.MAV_AUTOPILOT_INVALID


def heartbeat_armed(msg):
    return bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def populate_scaled_imu(imu, msg, *, flu=False):
    """Convert SCALED_IMU[2] into SI units, optionally changing FRD to FLU."""
    for axis, sign in (('x', 1), ('y', -1 if flu else 1), ('z', -1 if flu else 1)):
        setattr(imu.angular_velocity, axis, sign * units.milliradians_per_second_to_radians_per_second(
            getattr(msg, axis + 'gyro')))
        setattr(imu.linear_acceleration, axis, sign * units.milligravity_to_meters_per_second_squared(
            getattr(msg, axis + 'acc')))
