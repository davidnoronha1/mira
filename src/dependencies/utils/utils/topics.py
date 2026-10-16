"""Canonical ROS interface names shared by Mira and mira_sim.

Every Mira-owned topic and service is ``/mira/<domain>/<subject>[/<name>]``:

``sensors``     Raw device streams. Hardware drivers and the simulator bridge
                publish the same names, so the stack is identical in both.
``perception``  Vision worker outputs, ``/mira/perception/<camera>/<name>``.
``odometry``    Motion estimates forwarded to the autopilot EKF.
``state``       Autopilot state reported over MAVLink.
``guidance``    GUIDED-mode goals and hold.
``control``     Normalized effort commands and controller services.
``safety``      Emergency stop.
``sim``         Simulator-only data such as ground truth; never on the vehicle.
``replay``      Offline estimators run by diagnostic tools.

Names are lower_snake_case. ROS-standard interfaces (``/clock``, ``/tf``,
``/tf_static``, ``/diagnostics``, ``/joy``) and the competition bridge topics
keep their conventional names. Keep ``utils/topics.hpp`` in sync.
"""

ROOT = '/mira'


def camera(name, stream='image_raw'):
    """Raw stream of the named camera, e.g. ``camera('front', 'camera_info')``."""
    return f'{ROOT}/sensors/camera/{name}/{stream}'


def perception(worker):
    """Namespace of a perception worker; its outputs are relative to it."""
    return f'{ROOT}/perception/{worker}'


# sensors
FRONT_IMAGE = camera('front')
FRONT_CAMERA_INFO = camera('front', 'camera_info')
FRONT_DEPTH = camera('front', 'depth/image_raw')
BOTTOM_IMAGE = camera('bottom')
BOTTOM_CAMERA_INFO = camera('bottom', 'camera_info')
IMU = f'{ROOT}/sensors/imu'

# perception
FRONT_OBJECTS = f'{perception("front")}/objects'
FRONT_OBSTACLES = f'{perception("front")}/obstacles'
FRONT_GATE_POSE = f'{perception("front")}/gate_pose'
BOTTOM_DOCK_POSE = f'{perception("bottom")}/dock_pose'
BOTTOM_TARGET_POSE = f'{perception("bottom")}/target_pose'
BOTTOM_TARGET_COLOR = f'{perception("bottom")}/target_color'
BOTTOM_PIPELINE_OFFSET = f'{perception("bottom")}/pipeline/offset'
BOTTOM_PIPELINE_ANGLE = f'{perception("bottom")}/pipeline/angle'

# odometry
VISUAL_ODOMETRY = f'{ROOT}/odometry/visual'
FLOW_VELOCITY = f'{ROOT}/odometry/flow_velocity'
FLOOR_HEADING = f'{ROOT}/odometry/floor_heading'

# state
TELEMETRY = f'{ROOT}/state/telemetry'
STATE_IMU = f'{ROOT}/state/imu'
POSE = f'{ROOT}/state/pose'
ATTITUDE = f'{ROOT}/state/attitude'
DEPTH = f'{ROOT}/state/depth'
PATH = f'{ROOT}/state/path'

# guidance
GOAL = f'{ROOT}/guidance/goal'
GOAL_DIRECT = f'{ROOT}/guidance/goal_direct'
ACTIVE_GOAL = f'{ROOT}/guidance/active_goal'
HOLD = f'{ROOT}/guidance/hold'

# control
COMMAND = f'{ROOT}/control/command'
ARM = f'{ROOT}/control/arm'
CHANGE_MODE = f'{ROOT}/control/change_mode'
PIPELINE_ENABLE = f'{ROOT}/control/pipeline/enable'

# safety
EMERGENCY_STOP = f'{ROOT}/safety/emergency_stop'

# sim
GROUND_TRUTH_ODOM = f'{ROOT}/sim/ground_truth/odom'
GROUND_TRUTH_PATH = f'{ROOT}/sim/ground_truth/path'
FRONT_BOXES = f'{ROOT}/sim/camera/front/boxes'
THRUSTERS = f'{ROOT}/sim/thrusters'
MOTION_COMMAND = f'{ROOT}/sim/motion_command'
