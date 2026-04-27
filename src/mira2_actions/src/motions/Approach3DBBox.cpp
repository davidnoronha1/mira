#include "motions.hpp"
#include <cmath>

void Approach3DBBox::objects_callback(
    const zed_msgs::msg::ObjectsStamped::SharedPtr msg) {
  bool found_this_frame = false;
  double best_confidence = 0.0;

  for (const auto& obj : msg->objects) {
    if (obj.label != target_label_ && obj.sublabel != target_label_)
      continue;

    if (obj.confidence > best_confidence) {
      best_confidence = obj.confidence;
      target_pose_ = extractPoseFromObject(obj);
      target_visible_ = true;
      found_this_frame = true;
    }
  }

  if (found_this_frame) {
    last_detection_time_ = ros_state_->node->now();
    RCLCPP_INFO_THROTTLE(ros_state_->node->get_logger(),
                         *ros_state_->node->get_clock(), 1000,
                         "Approach3DBBox: Found '%s' at (%.2f, %.2f, %.2f)",
                         target_label_.c_str(),
                         target_pose_.position.x,
                         target_pose_.position.y,
                         target_pose_.position.z);
  } else if (!msg->objects.empty()) {
    RCLCPP_WARN_THROTTLE(
        ros_state_->node->get_logger(), *ros_state_->node->get_clock(), 2000,
        "Approach3DBBox: Received %zu objects, but none match '%s'",
        msg->objects.size(), target_label_.c_str());
  }
}

Approach3DBBox::Approach3DBBox(const std::string& name,
                           const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state),
      lateral_pid(name + "_lat", ros_state->node),
      forward_pid(name + "_fwd", ros_state->node),
      depth_pid(name + "_depth", ros_state->node),
      yaw_pid(name + "_yaw", ros_state->node),
      target_visible_(false), heading_locked_(false) {}

BT::PortsList Approach3DBBox::providedPorts() {
  return {
      BT::InputPort<std::string>("label", "Target object label to approach"),
      BT::InputPort<double>("success_distance", 0.5,
                          "Distance to target to succeed (meters)"),
      BT::InputPort<double>("approach_distance", 1.0,
                          "Distance to stop at before final approach"),
      BT::InputPort<double>("object_lost_timeout", 3.0,
                          "Seconds after which FAILURE if object lost"),
      BT::InputPort<double>("timeout", 60.0,
                          "Maximum time for entire approach (seconds)"),
      BT::InputPort<std::string>("flight_mode", "STABILIZE",
                            "FC mode: ALT_HOLD or STABILIZE"),

      BT::InputPort<double>("lateral_pid_kp", 1.0, "Lateral PID kp"),
      BT::InputPort<double>("lateral_pid_ki", 0.002, "Lateral PID ki"),
      BT::InputPort<double>("lateral_pid_kd", 2.0, "Lateral PID kd"),
      BT::InputPort<double>("lateral_pid_base_offset", 1500.0, "Lateral PID base"),

      BT::InputPort<double>("forward_pid_kp", 1.0, "Forward PID kp"),
      BT::InputPort<double>("forward_pid_ki", 0.002, "Forward PID ki"),
      BT::InputPort<double>("forward_pid_kd", 2.0, "Forward PID kd"),
      BT::InputPort<double>("forward_pid_base_offset", 1500.0, "Forward PID base"),

      BT::InputPort<double>("depth_pid_kp", 0.05, "Depth PID kp"),
      BT::InputPort<double>("depth_pid_ki", 0.005, "Depth PID ki"),
      BT::InputPort<double>("depth_pid_kd", 20.0, "Depth PID kd"),
      BT::InputPort<double>("depth_pid_base_offset", 1580.0, "Depth PID base"),

      BT::InputPort<double>("yaw_pid_kp", 1.5, "Yaw PID kp"),
      BT::InputPort<double>("yaw_pid_ki", 0.005, "Yaw PID ki"),
      BT::InputPort<double>("yaw_pid_kd", 3.0, "Yaw PID kd"),
      BT::InputPort<double>("yaw_pid_base_offset", 1500.0, "Yaw PID base"),
  };
}

geometry_msgs::msg::Pose Approach3DBBox::extractPoseFromObject(
    const zed_msgs::msg::Object& obj) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = obj.position[0];
  pose.position.y = obj.position[1];
  pose.position.z = obj.position[2];

  if (obj.bounding_box_3d.corners.size() >= 8) {
    tf2::Quaternion q = extractRotationFromCorners(obj.bounding_box_3d.corners);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
  } else {
    pose.orientation.w = 1.0;
  }

  return pose;
}

tf2::Quaternion Approach3DBBox::extractRotationFromCorners(
    const std::array<zed_msgs::msg::Keypoint3D, 8>& corners) {
  if (corners.size() < 8) {
    return tf2::Quaternion::getIdentity();
  }

  auto kp = [&](const zed_msgs::msg::Keypoint3D& c, int i) { return c.kp[i]; };

  tf2::Vector3 v3(
      kp(corners[4], 0) - kp(corners[0], 0),
      kp(corners[4], 1) - kp(corners[0], 1),
      kp(corners[4], 2) - kp(corners[0], 2));
  tf2::Vector3 v1(
      kp(corners[1], 0) - kp(corners[0], 0),
      kp(corners[1], 1) - kp(corners[0], 1),
      kp(corners[1], 2) - kp(corners[0], 2));
  tf2::Vector3 v2_temp(
      kp(corners[3], 0) - kp(corners[0], 0),
      kp(corners[3], 1) - kp(corners[0], 1),
      kp(corners[3], 2) - kp(corners[0], 2));

  v3.normalize();
  v1.normalize();
  tf2::Vector3 v2 = v3.cross(v1);
  v2.normalize();
  v1 = v2.cross(v3);
  v1.normalize();

  tf2::Matrix3x3 rot_matrix(
      v3.x(), v1.x(), v2.x(),
      v3.y(), v1.y(), v2.y(),
      v3.z(), v1.z(), v2.z());

  tf2::Quaternion q;
  rot_matrix.getRotation(q);
  return q.normalize();
}

geometry_msgs::msg::Pose Approach3DBBox::computeApproachPose(
    const geometry_msgs::msg::Pose& object_pose,
    const tf2::Quaternion& object_orientation,
    double distance) {

  tf2::Matrix3x3 rot_matrix(object_orientation);
  tf2::Vector3 obj_x = rot_matrix.getColumn(0);

  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x = object_pose.position.x - distance * obj_x.x();
  approach_pose.position.y = object_pose.position.y - distance * obj_x.y();
  approach_pose.position.z = object_pose.position.z;

  tf2::Quaternion approach_q = object_orientation.inverse();
  approach_pose.orientation.x = approach_q.x();
  approach_pose.orientation.y = approach_q.y();
  approach_pose.orientation.z = approach_q.z();
  approach_pose.orientation.w = approach_q.w();

  return approach_pose;
}

BT::NodeStatus Approach3DBBox::onStart() {
  auto label = getInput<std::string>("label");
  if (!label) {
    RCLCPP_ERROR(ros_state_->node->get_logger(),
                "Approach3DBBox: missing required input [label]: %s",
                label.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  target_label_ = label.value();
  success_distance_ = getInput<double>("success_distance").value();
  approach_distance_ = getInput<double>("approach_distance").value();
  object_lost_timeout_ = getInput<double>("object_lost_timeout").value();
  timeout_ = getInput<double>("timeout").value();
  flight_mode_ = getInput<std::string>("flight_mode").value();

  lateral_pid.kp = getInput<double>("lateral_pid_kp").value();
  lateral_pid.ki = getInput<double>("lateral_pid_ki").value();
  lateral_pid.kd = getInput<double>("lateral_pid_kd").value();
  lateral_pid.base_offset = getInput<double>("lateral_pid_base_offset").value();
  lateral_pid.emptyError();

  forward_pid.kp = getInput<double>("forward_pid_kp").value();
  forward_pid.ki = getInput<double>("forward_pid_ki").value();
  forward_pid.kd = getInput<double>("forward_pid_kd").value();
  forward_pid.base_offset = getInput<double>("forward_pid_base_offset").value();
  forward_pid.emptyError();

  depth_pid.kp = getInput<double>("depth_pid_kp").value();
  depth_pid.ki = getInput<double>("depth_pid_ki").value();
  depth_pid.kd = getInput<double>("depth_pid_kd").value();
  depth_pid.base_offset = getInput<double>("depth_pid_base_offset").value();
  depth_pid.emptyError();

  yaw_pid.kp = getInput<double>("yaw_pid_kp").value();
  yaw_pid.ki = getInput<double>("yaw_pid_ki").value();
  yaw_pid.kd = getInput<double>("yaw_pid_kd").value();
  yaw_pid.base_offset = getInput<double>("yaw_pid_base_offset").value();
  yaw_pid.emptyError();

  start_time_ = ros_state_->node->now();
  last_detection_time_ = start_time_;
  target_visible_ = false;

  RCLCPP_INFO(ros_state_->node->get_logger(),
              "Approach3DBBox: starting to approach '%s' "
              "(success_dist=%.2fm, approach_dist=%.2fm)",
              target_label_.c_str(), success_distance_, approach_distance_);

  objects_sub_ = ros_state_->node->create_subscription<zed_msgs::msg::ObjectsStamped>(
      "/zed/obj_det/objects", 10,
      std::bind(&Approach3DBBox::objects_callback, this, std::placeholders::_1));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Approach3DBBox::onRunning() {
  rclcpp::Time now = ros_state_->node->now();
  double elapsed = (now - start_time_).seconds();

  if (elapsed > timeout_) {
    RCLCPP_WARN(ros_state_->node->get_logger(),
                "Approach3DBBox: hard timeout after %.1f s", elapsed);
    publish_neutral();
    return BT::NodeStatus::FAILURE;
  }

  double time_since_detection = (now - last_detection_time_).seconds();
  if (elapsed > 1.0 && time_since_detection > object_lost_timeout_) {
    RCLCPP_WARN(ros_state_->node->get_logger(),
                "Approach3DBBox: object '%s' lost for %.1f s -> FAILURE",
                target_label_.c_str(), time_since_detection);
    publish_neutral();
    return BT::NodeStatus::FAILURE;
  }

  if (!target_visible_) {
    RCLCPP_INFO_THROTTLE(ros_state_->node->get_logger(),
                         *ros_state_->node->get_clock(), 1000,
                         "Approach3DBBox: [WAITING] no '%s' detected",
                         target_label_.c_str());
    publish_neutral();
    return BT::NodeStatus::RUNNING;
  }

  double obj_x = target_pose_.position.x;
  double obj_y = target_pose_.position.y;
  double obj_z = target_pose_.position.z;
  double distance = std::sqrt(obj_x * obj_x + obj_y * obj_y + obj_z * obj_z);

  if (distance <= success_distance_) {
    RCLCPP_INFO(ros_state_->node->get_logger(),
                "Approach3DBBox: distance %.2fm <= %.2fm -> SUCCESS",
                distance, success_distance_);
    publish_neutral();
    return BT::NodeStatus::SUCCESS;
  }

  if (!heading_locked_) {
    locked_heading_ = ros_state_->telemetry.yaw;
    heading_locked_ = true;
  }

  tf2::Quaternion obj_q(
      target_pose_.orientation.x,
      target_pose_.orientation.y,
      target_pose_.orientation.z,
      target_pose_.orientation.w);

  tf2::Vector3 obj_forward(1.0, 0.0, 0.0);
  if (obj_q.length2() > 0.001) {
    obj_q.normalize();
    tf2::Matrix3x3 rot(obj_q);
    obj_forward = rot.getColumn(0);
  }

  double forward_error = obj_forward.x() * obj_x + obj_forward.y() * obj_y + obj_forward.z() * obj_z;
  double lateral_error = -obj_forward.y() * obj_x + obj_forward.x() * obj_y;
  double depth_error = -obj_z;

  double yaw_error = locked_heading_ - ros_state_->telemetry.yaw;
  while (yaw_error > 180.0) yaw_error -= 360.0;
  while (yaw_error < -180.0) yaw_error += 360.0;

  float lateral_pwm = lateral_pid.pid_control(lateral_error, elapsed, false);
  float forward_pwm = forward_pid.pid_control(forward_error, elapsed, false);
  float depth_pwm = depth_pid.pid_control(depth_error, elapsed, false);
  float yaw_pwm = yaw_pid.pid_control(yaw_error, elapsed, false);

  custom_msgs::msg::Commands cmd;
  cmd.mode = flight_mode_;
  cmd.arm = true;
  cmd.forward = static_cast<int>(forward_pwm);
  cmd.lateral = static_cast<int>(lateral_pwm);
  cmd.thrust = static_cast<int>(depth_pwm);
  cmd.yaw = static_cast<int>(yaw_pwm);
  ros_state_->cmd_publisher->publish(cmd);

  RCLCPP_INFO_THROTTLE(ros_state_->node->get_logger(),
                       *ros_state_->node->get_clock(), 1000,
                       "Approach3DBBox: dist=%.2f fwd_err=%.2f lat_err=%.2f "
                       "-> Fwd: %d, Lat: %d, Thr: %d, Yaw: %d",
                       distance, forward_error, lateral_error,
                       cmd.forward, cmd.lateral, cmd.thrust, cmd.yaw);

  return BT::NodeStatus::RUNNING;
}

void Approach3DBBox::onHalted() {
  RCLCPP_INFO(ros_state_->node->get_logger(), "Approach3DBBox: halted");
  publish_neutral();
}

void Approach3DBBox::publish_neutral() {
  custom_msgs::msg::Commands cmd;
  cmd.mode = flight_mode_;
  cmd.arm = true;
  cmd.forward = 1500;
  cmd.lateral = 1500;
  cmd.thrust = 1500;
  cmd.yaw = 1500;
  ros_state_->cmd_publisher->publish(cmd);
}