"""Normalized body-effort commands and calibrated RC overrides."""
import math

import utils.time as utime
from custom_msgs.msg import CommandsNormalized
from custom_msgs.srv import ChangeMode
from std_srvs.srv import SetBool
from utils.control import COMMAND_FIELDS
from utils import topics

from mira2_hardware.config import load_command_calibration
from mira2_hardware.controller_plugin import ControllerPlugin


class RCControl(ControllerPlugin):
    GUIDED_MODE = 4
    COMMAND_TIMEOUT_SECONDS = 0.5
    COMMAND_INPUT_TOPIC = topics.COMMAND
    RC_CHANNEL_FIELDS = ('pitch', 'roll', 'thrust', 'yaw', 'forward', 'lateral')
    UNUSED_RC_CHANNEL = 65535

    def __init__(self, controller):
        super().__init__(controller)
        self.calibration = load_command_calibration(controller)
        self.initial_mode = self.get_parameter('initial_mode').value
        self.overrides_active = False
        self.last_command = None
        self.update_channels()
        self.create_subscription(CommandsNormalized, self.COMMAND_INPUT_TOPIC, self.command_callback, 1)
        self.create_service(SetBool, topics.ARM, self.set_arm)
        self.create_service(ChangeMode, topics.CHANGE_MODE, self.set_mode)

    def on_connect(self):
        self.last_command = None
        self.update_channels()
        if not self.arm_state and self.mode != self.GUIDED_MODE:
            self.update_fcu(mode=self.initial_mode, force_mode=True)

    def on_message(self, msg):
        if msg.get_type() == 'HEARTBEAT' and self.mode == self.GUIDED_MODE:
            self.last_command = None
            self.update_channels()
            if self.overrides_active:
                # Release the eight RC overrides previously owned by this plugin.
                self.master.mav.rc_channels_override_send(
                    self.master.target_system, self.master.target_component, *([0] * 8))
                self.overrides_active = False

    def tick(self, now):
        if self.mode == self.GUIDED_MODE:
            return
        if self.last_command is not None and utime.elapsed(self.last_command, now) > self.COMMAND_TIMEOUT_SECONDS:
            self.stop('Command input timed out')
        self.actuate()

    def update_channels(self, values=None):
        """Store calibrated RC channels, or neutral channels when values are absent."""
        if values is None or values == 0:
            channels = [self.calibration[field].neutral for field in self.RC_CHANNEL_FIELDS]
        else:
            channels = [self.calibration[field].to_pwm(values[field])
                        for field in self.RC_CHANNEL_FIELDS]
        self.channels = channels + [self.UNUSED_RC_CHANNEL, self.UNUSED_RC_CHANNEL]

    def command_callback(self, msg: CommandsNormalized):
        if not self.ready or self.emergency_active or self.mode == self.GUIDED_MODE:
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
        self.update_fcu(armed=bool(msg.arm))
        if msg.mode:
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
            self.request_arm(armed)

        if mode:
            mapping = self.master.mode_mapping() or {}
            if mode not in mapping:
                return False
            if force_mode or mapping[mode] != self.mode:
                self.request_mode(mapping[mode])

        return True

    def stop(self, reason):
        self.update_channels()
        self.last_command = None
        self.update_fcu(armed=False)
        self.actuate()
        self.get_logger().warn(reason)

    def set_arm(self, request, response):
        if not self.ready:
            response.success = False
            response.message = "Autopilot not ready"
        elif request.data and self.emergency_active:
            response.success = False
            response.message = "Emergency stop is active"
        elif request.data and self.mode == self.GUIDED_MODE:
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

    def actuate(self):
        if self.mode == self.GUIDED_MODE:
            return
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component, *self.channels)
        self.overrides_active = True

