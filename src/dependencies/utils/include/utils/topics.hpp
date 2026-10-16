#pragma once

// Canonical ROS interface names; the schema is documented in utils/topics.py.
// Keep both files in sync.
namespace utils::topics {

// sensors
inline constexpr const char *FRONT_IMAGE = "/mira/sensors/camera/front/image_raw";
inline constexpr const char *FRONT_CAMERA_INFO = "/mira/sensors/camera/front/camera_info";
inline constexpr const char *FRONT_DEPTH = "/mira/sensors/camera/front/depth/image_raw";
inline constexpr const char *BOTTOM_IMAGE = "/mira/sensors/camera/bottom/image_raw";
inline constexpr const char *BOTTOM_CAMERA_INFO = "/mira/sensors/camera/bottom/camera_info";
inline constexpr const char *IMU = "/mira/sensors/imu";

// perception
inline constexpr const char *FRONT_OBJECTS = "/mira/perception/front/objects";
inline constexpr const char *FRONT_OBSTACLES = "/mira/perception/front/obstacles";
inline constexpr const char *FRONT_GATE_POSE = "/mira/perception/front/gate_pose";
inline constexpr const char *BOTTOM_DOCK_POSE = "/mira/perception/bottom/dock_pose";
inline constexpr const char *BOTTOM_TARGET_POSE = "/mira/perception/bottom/target_pose";
inline constexpr const char *BOTTOM_TARGET_COLOR = "/mira/perception/bottom/target_color";
inline constexpr const char *BOTTOM_PIPELINE_OFFSET = "/mira/perception/bottom/pipeline/offset";
inline constexpr const char *BOTTOM_PIPELINE_ANGLE = "/mira/perception/bottom/pipeline/angle";

// odometry
inline constexpr const char *VISUAL_ODOMETRY = "/mira/odometry/visual";
inline constexpr const char *FLOW_VELOCITY = "/mira/odometry/flow_velocity";
inline constexpr const char *FLOOR_HEADING = "/mira/odometry/floor_heading";

// state
inline constexpr const char *TELEMETRY = "/mira/state/telemetry";
inline constexpr const char *STATE_IMU = "/mira/state/imu";
inline constexpr const char *POSE = "/mira/state/pose";
inline constexpr const char *ATTITUDE = "/mira/state/attitude";
inline constexpr const char *DEPTH = "/mira/state/depth";
inline constexpr const char *PATH = "/mira/state/path";

// guidance
inline constexpr const char *GOAL = "/mira/guidance/goal";
inline constexpr const char *GOAL_DIRECT = "/mira/guidance/goal_direct";
inline constexpr const char *ACTIVE_GOAL = "/mira/guidance/active_goal";
inline constexpr const char *HOLD = "/mira/guidance/hold";

// control
inline constexpr const char *COMMAND = "/mira/control/command";
inline constexpr const char *ARM = "/mira/control/arm";
inline constexpr const char *CHANGE_MODE = "/mira/control/change_mode";
inline constexpr const char *PIPELINE_ENABLE = "/mira/control/pipeline/enable";

// safety
inline constexpr const char *EMERGENCY_STOP = "/mira/safety/emergency_stop";

// sim
inline constexpr const char *GROUND_TRUTH_ODOM = "/mira/sim/ground_truth/odom";
inline constexpr const char *GROUND_TRUTH_PATH = "/mira/sim/ground_truth/path";
inline constexpr const char *FRONT_BOXES = "/mira/sim/camera/front/boxes";
inline constexpr const char *CHASE_IMAGE = "/mira/sim/camera/chase/image_raw";
inline constexpr const char *THRUSTERS = "/mira/sim/thrusters";
inline constexpr const char *MOTION_COMMAND = "/mira/sim/motion_command";

}  // namespace utils::topics
