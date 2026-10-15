"""ROS-independent command calibration; PWM belongs at the hardware boundary."""

from dataclasses import dataclass
import math


COMMAND_FIELDS = ("forward", "lateral", "thrust", "pitch", "roll", "yaw")


def normalized(value):
    """Reject nonfinite commands, then saturate finite effort to [-1, 1]."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Command values must be finite")
    return max(-1.0, min(1.0, value))


@dataclass(frozen=True)
class PWMCalibration:
    """Piecewise-linear calibration for one bidirectional channel.

    Does not infer frames or actuator type. Inversion must be specified by the
    vehicle configuration. Unidirectional actuators need a separate contract.
    """

    minimum: int = 1100
    neutral: int = 1500
    maximum: int = 1900
    inverted: bool = False
    deadband: float = 0.0

    def __post_init__(self):
        if not all(type(value) is int for value in (self.minimum, self.neutral, self.maximum)):
            raise ValueError("PWM endpoints must be integers")
        if not 0 < self.minimum < self.neutral < self.maximum < 65535:
            raise ValueError("PWM calibration requires 0 < minimum < neutral < maximum < 65535")
        if not math.isfinite(self.deadband) or not 0 <= self.deadband < 1:
            raise ValueError("Deadband must be finite and in [0, 1)")
        if type(self.inverted) is not bool:
            raise ValueError("Inversion must be boolean")

    def to_pwm(self, value):
        value = normalized(value)
        if abs(value) <= self.deadband:
            value = 0.0
        if self.inverted:
            value = -value
        span = self.maximum - self.neutral if value >= 0 else self.neutral - self.minimum
        return round(self.neutral + value * span)

    def from_pwm(self, value):
        """Migration helper, not a source of hardware constants in controllers."""
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("PWM values must be finite")
        span = self.maximum - self.neutral if value >= self.neutral else self.neutral - self.minimum
        effort = normalized((value - self.neutral) / span)
        return -effort if self.inverted else effort


class PIDController:
    """Reusable, bounded normalized-effort PID, independent of ROS and GUI."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, bias=0.0, limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.bias, self.limit = bias, limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.terms = (0.0, 0.0, 0.0)

    def control(self, error, dt):
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0:
            self.reset()
            return 0.0
        if not all(math.isfinite(value) for value in
                   (self.kp, self.ki, self.kd, self.bias, self.limit)) or not 0 < self.limit <= 1:
            raise ValueError("PID gains must be finite and limit must be in (0, 1]")
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        candidate = self.integral + error * dt
        p, d = self.kp * error, self.kd * derivative
        output = self.bias + p + self.ki * candidate + d
        # Conditional integration: permit unwinding, not further saturation.
        if abs(output) <= self.limit or error * output < 0:
            self.integral = candidate
        self.previous_error = error
        self.terms = (p, self.ki * self.integral, d)
        return max(-self.limit, min(self.limit, self.bias + sum(self.terms)))


def publish_mission_command(node, publisher, command):
    """Stamp a copy and map retained right/clockwise mission axes to body FLU."""
    from copy import deepcopy
    out = deepcopy(command)
    out.header.stamp = node.get_clock().now().to_msg()
    out.header.frame_id = "base_link"
    out.lateral, out.pitch, out.yaw = -out.lateral, -out.pitch, -out.yaw
    publisher.publish(out)
