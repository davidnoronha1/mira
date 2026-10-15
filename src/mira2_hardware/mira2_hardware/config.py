"""ROS parameter schema; deployment overrides live in config/*.yaml."""

from utils.parameters import dataclass_config, declare_config
from utils.control import COMMAND_FIELDS, PWMCalibration


# MAVLink connection parameters.
CONNECTION_DEFAULTS = {
    "pixhawk_address": "/dev/Pixhawk",
    "baudrate": 57600,
}

# Flight-controller startup parameters.
STARTUP_DEFAULTS = {
    "initial_mode": "STABILIZE",
}

DEFAULTS = {
    **CONNECTION_DEFAULTS,
    **STARTUP_DEFAULTS,
}


def load_master_config(node):
    """Declare connection/startup parameters and load per-axis calibration."""
    declare_config(node, DEFAULTS)
    calibration = {}

    # Load each axis; inversion is specified in command_calibration.yaml.
    for axis in COMMAND_FIELDS:
        calibration[axis] = dataclass_config(
            node,
            PWMCalibration(),
            namespace=axis,
        )
    return calibration
