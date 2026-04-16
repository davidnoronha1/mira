#include "motions.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

void ApproachBB::bbox_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
  bb_found_ = false;
  bb_area_norm_ = 0.0;
  bool found_this_frame = false;
  for (const auto &detection : msg->detections) {
    if (detection.id != target_object_)
      continue;

    found_this_frame = true;
    auto bbox = detection.bbox;

    double x_center_norm = bbox.center.position.x / frame_width_;
    double y_center_norm = bbox.center.position.y / frame_height_;

    double bb_area_pixels = bbox.size_x * bbox.size_y;
    double frame_area = frame_width_ * frame_height_;
    double bb_area = bb_area_pixels / frame_area;

    if (!bb_found_ || bb_area > bb_area_norm_) {
      bb_found_ = true;
      bb_x_center_norm_ = x_center_norm;
      bb_y_center_norm_ = y_center_norm;
      bb_area_norm_ = bb_area;
    }
  }

  if (found_this_frame) {
    last_detection_time_ = ros_state_->node->now();
    RCLCPP_INFO_THROTTLE(ros_state_->node->get_logger(), *ros_state_->node->get_clock(), 1000,
        "ApproachBB: Found '%s' (area=%.1f%%, x=%.2f, y=%.2f)",
        target_object_.c_str(), bb_area_norm_ * 100.0, bb_x_center_norm_, bb_y_center_norm_);
  } else if (!msg->detections.empty()) {
     RCLCPP_WARN_THROTTLE(ros_state_->node->get_logger(), *ros_state_->node->get_clock(), 2000,
        "ApproachBB: Received %zu detections, but none match '%s'",
        msg->detections.size(), target_object_.c_str());
  }
}

ApproachBB::ApproachBB(const std::string &name,
                       const BT::NodeConfiguration &config, ROSState *ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state),
      lateral_pid("lateral_gtbb", ros_state->node),
      depth_pid("depth_gtbb", ros_state->node),
      yaw_pid("yaw_lock_gtbb", ros_state->node), forward_pwm_(1550.0),
      frame_width_(1.0), frame_height_(1.0), success_bb_area_(0.70),
      bb_lost_timeout_(1.0), timeout_(20.0), flight_mode_("MANUAL"),
      bb_found_(false), bb_x_center_norm_(0.5), bb_y_center_norm_(0.8),
      bb_area_norm_(0.0), depth_setpoint_base_(0.0), depth_visual_gain_(10.0),
      locked_heading_(0.0) {
  // Subscription is now created in onStart() to ensure parameters are ready.
}

BT::PortsList ApproachBB::providedPorts() {
  return {
      BT::InputPort<std::string>("object", "Target Object label to approach "),
      BT::InputPort<double>("forward_pwm", 1500.0,
                            "PWM value for forward movement, default 1550"),
      // Frame resolution — set these if your vision node gives raw pixels.
      // Leave at 1.0 if it already gives normalized [0,1] coordinates.
      BT::InputPort<double>(
          "frame_width", 832.0,
          "Camera frame width in pixels (or 1.0 if normalized)"),
      BT::InputPort<double>(
          "frame_height", 464.0,
          "Camera frame height in pixels (or 1.0 if normalized)"),

      // Success condition
      BT::InputPort<double>(
          "success_area_ratio", 0.70,
          "Succeed when BB area >= this fraction of frame area (0.0-1.0). "
          "0.70 means object fills 70% of screen → you are close enough."),

      // Loss detection
      BT::InputPort<double>(
          "object_lost_timeout", 3.0,
          "Seconds after which FAILURE is declared if BB disappears"),

      // Hard timeout
      BT::InputPort<double>(
          "timeout", 60.0,
          "Maximum time for entire approach before FAILURE (seconds)"),

      // Flight controller mode
      BT::InputPort<std::string>(
          "flight_mode", "MANUAL",
          "FC mode: ALT_HOLD (auto depth hold) or STABILIZE (manual thrust)"),

      // Depth PID — adjusts thrust to hold depth, setpoint shifted by visual Y
      // error
      BT::InputPort<double>(
          "depth_setpoint", 0.0,
          "Baseline pressure setpoint (same units as external_pressure)"),
      BT::InputPort<double>(
          "depth_visual_gain", 10.0,
          "Gain scaling how much Y-error shifts the depth setpoint"),
      BT::InputPort<double>("depth_pid_kp", 0.05,
                            "Depth PID proportional gain"),
      BT::InputPort<double>("depth_pid_ki", 0.005, "Depth PID integral gain"),
      BT::InputPort<double>("depth_pid_kd", 20.20, "Depth PID derivative gain"),
      BT::InputPort<double>("depth_pid_base_offset", 1580.0,
                            "Depth PID base offset (neutral thrust PWM)"),

      // Yaw lock PID — holds initial heading against rotational disturbances
      BT::InputPort<double>("yaw_pid_kp", 1.5,
                            "Yaw lock PID proportional gain"),
      BT::InputPort<double>("yaw_pid_ki", 0.005, "Yaw lock PID integral gain"),
      BT::InputPort<double>("yaw_pid_kd", 3.0, "Yaw lock PID derivative gain"),
      BT::InputPort<double>("yaw_pid_base_offset", 1500.0,
                            "Yaw lock PID base offset (neutral PWM)"),

      // Lateral PID — strafes the vehicle to center BB horizontally
      BT::InputPort<double>("lateral_pid_kp", 1.0,
                            "Lateral PID proportional gain"),
      BT::InputPort<double>("lateral_pid_ki", 0.002,
                            "Lateral PID integral gain"),
      BT::InputPort<double>("lateral_pid_kd", 2.0,
                            "Lateral PID derivative gain"),
      BT::InputPort<double>("lateral_pid_base_offset", 1500.0,
                            "Lateral PID base offset (neutral PWM)"),
  };
}

BT::NodeStatus ApproachBB::onStart() {
  auto object = getInput<std::string>("object");
  if (!object) {
    RCLCPP_ERROR(ros_state_->node->get_logger(),
                 "ApproachBB: missing required input [object]: %s", object.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  target_object_ = object.value();
  forward_pwm_ = getInput<double>("forward_pwm").value();
  frame_width_ = getInput<double>("frame_width").value();
  frame_height_ = getInput<double>("frame_height").value();
  success_bb_area_ = getInput<double>("success_area_ratio").value();
  bb_lost_timeout_ = getInput<double>("object_lost_timeout").value();
  timeout_ = getInput<double>("timeout").value();
  flight_mode_ = getInput<std::string>("flight_mode").value();

  if (frame_width_ <= 0.0) {
    RCLCPP_ERROR(ros_state_->node->get_logger(),
                 "ApproachBB: [frame_width] must be > 0, got %f", frame_width_);
    return BT::NodeStatus::FAILURE;
  }
  if (frame_height_ <= 0.0) {
    RCLCPP_ERROR(ros_state_->node->get_logger(),
                 "ApproachBB: [frame_height] must be > 0, got %f", frame_height_);
    return BT::NodeStatus::FAILURE;
  }
  if (success_bb_area_ <= 0.0) {
    RCLCPP_ERROR(ros_state_->node->get_logger(),
                 "ApproachBB: [success_area_ratio] must be > 0, got %f", success_bb_area_);
    return BT::NodeStatus::FAILURE;
  }
  // Lock heading at start
  locked_heading_ = ros_state_->telemetry.yaw;

  // Configure PIDs
  yaw_pid.kp = getInput<double>("yaw_pid_kp").value();
  yaw_pid.ki = getInput<double>("yaw_pid_ki").value();
  yaw_pid.kd = getInput<double>("yaw_pid_kd").value();
  yaw_pid.base_offset = getInput<double>("yaw_pid_base_offset").value();
  yaw_pid.emptyError();

  depth_setpoint_base_ = getInput<double>("depth_setpoint").value();
  depth_visual_gain_ = getInput<double>("depth_visual_gain").value();

  depth_pid.kp = getInput<double>("depth_pid_kp").value();
  depth_pid.ki = getInput<double>("depth_pid_ki").value();
  depth_pid.kd = getInput<double>("depth_pid_kd").value();
  depth_pid.base_offset = getInput<double>("depth_pid_base_offset").value();
  depth_pid.emptyError();

  lateral_pid.kp = getInput<double>("lateral_pid_kp").value();
  lateral_pid.ki = getInput<double>("lateral_pid_ki").value();
  lateral_pid.kd = getInput<double>("lateral_pid_kd").value();
  lateral_pid.base_offset = getInput<double>("lateral_pid_base_offset").value();
  lateral_pid.emptyError();

  start_time_ = ros_state_->node->now();
  last_detection_time_ = start_time_;
  bb_found_ = false;
  bb_area_norm_ = 0.0;

  RCLCPP_INFO(ros_state_->node->get_logger(),
              "ApproachBB: starting to approach to '%s' "
              "(success at bb_area=%.0f%%, resolution=%.0fx%.0f, flight_mode=%s)",
              target_object_.c_str(), success_bb_area_ * 100.0,
              frame_width_, frame_height_,
              flight_mode_.c_str());

  bb_sub_ =
      ros_state_->node->create_subscription<vision_msgs::msg::Detection2DArray>(
          "/vision/detections", 10,
          std::bind(&ApproachBB::bbox_callback, this, std::placeholders::_1));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachBB::onRunning() {
  rclcpp::Time now = ros_state_->node->now();
  double elapsed = (now - start_time_).seconds();

  // ── 1. Hard timeout check ─────────────────────────────────────────────
  if (elapsed > timeout_) {
    RCLCPP_WARN(ros_state_->node->get_logger(),
                "ApproachBoundingBox: hard timeout after %.1f s", elapsed);
    publish_neutral();
    return BT::NodeStatus::FAILURE;
  }

  // ── 2. Object lost timeout ────────────────────────────────────────────
  // If the BB hasn't been seen for object_lost_timeout_ seconds → FAILURE.
  // This handles: object moved out of frame, vision node crashed, etc.
  double time_since_detection = (now - last_detection_time_).seconds();
  if (elapsed > 1.0 && time_since_detection > bb_lost_timeout_) {
    // The elapsed > 1.0 guard avoids false triggers right at startup
    // before the first detection has had a chance to arrive.
    RCLCPP_WARN(ros_state_->node->get_logger(),
                "ApproachBoundingBox: object '%s' lost for %.1f s → FAILURE",
                target_object_.c_str(), time_since_detection);
    publish_neutral();
    return BT::NodeStatus::FAILURE;
  }

  // ── 3. BB not visible in latest frame → hold and wait ────────────────
  if (!bb_found_) {
    RCLCPP_INFO_THROTTLE(ros_state_->node->get_logger(), *ros_state_->node->get_clock(), 1000,
                 "ApproachBoundingBox: [WAITING] no '%s' detected (%.1f s since last seen)",
                 target_object_.c_str(), time_since_detection);
    publish_neutral();
    return BT::NodeStatus::RUNNING;
  }
  // ── 4. Success condition ──────────────────────────────────────────────
  // BB area as fraction of frame area >= threshold means we are close enough.
  // e.g. success_bb_area_ = 0.70 → BB fills 70% of screen → SUCCESS.
  if (bb_area_norm_ >= success_bb_area_) {
    RCLCPP_INFO(ros_state_->node->get_logger(),
                "ApproachBoundingBox: reached '%s' (bb_area=%.1f%% >= %.1f%%)",
                target_object_.c_str(), bb_area_norm_ * 100.0,
                success_bb_area_ * 100.0);
    publish_neutral();
    return BT::NodeStatus::SUCCESS;
  }

  // ── 5. Compute horizontal centering error ─────────────────────────────
  // x_error is positive when BB center is to the RIGHT of screen center.
  // x_error is negative when BB center is to the LEFT of screen center.
  // Range: [-0.5, +0.5] in normalized coordinates.
  //
  //   screen: [0.0 ──── 0.5 (center) ──── 1.0]
  //   error = bb_x_center - 0.5
  //   error > 0 → BB is right of center → yaw right, strafe right
  //   error < 0 → BB is left of center  → yaw left,  strafe left
  double x_error = bb_x_center_norm_ - 0.5;

  // ── 6. PID outputs ────────────────────────────────────────────────────
  // Lateral PID strafes the vehicle to center the BB horizontally.
  float lateral_pwm = lateral_pid.pid_control(x_error, elapsed, false);

  // Depth PID:
  //   y_error > 0 → bbox is below the 80% line → vehicle needs to go shallower
  //   y_error < 0 → bbox is above the 80% line → vehicle needs to go deeper
  // Shift the depth setpoint proportionally so pressure PID closes the loop.
  double y_error = bb_y_center_norm_ - 0.8;
  double adjusted_depth_setpoint =
      depth_setpoint_base_ + depth_visual_gain_ * y_error;
  double depth_error =
      ros_state_->telemetry.external_pressure - adjusted_depth_setpoint;
  float thrust_pwm = depth_pid.pid_control(depth_error, elapsed, false);

  // ── 7. Determine forward PWM ──────────────────────────────────────────
  // Surge scales down linearly as the bbox fills the frame.
  // At bb_area=0 (far away) → full forward_pwm_.
  // At bb_area=success_bb_area_ (close enough) → neutral (1500).
  double area_factor = 1.0 - (bb_area_norm_ / success_bb_area_);
  area_factor = std::max(0.0, std::min(1.0, area_factor));
  double effective_forward = 1500.0 + (forward_pwm_ - 1500.0) * area_factor;

  // ── 8. Yaw lock ───────────────────────────────────────────────────────
  // Hold the heading recorded at onStart(). Normalize to [-180, +180]
  // to avoid spikes when crossing the 0°/360° boundary.
  double yaw_error = locked_heading_ - ros_state_->telemetry.yaw;
  while (yaw_error > 180.0)
    yaw_error -= 360.0;
  while (yaw_error < -180.0)
    yaw_error += 360.0;
  float yaw_pwm = yaw_pid.pid_control(yaw_error, elapsed, false);

  // ── 9. Publish command ────────────────────────────────────────────────
  custom_msgs::msg::Commands cmd;
  cmd.mode = flight_mode_; // "ALT_HOLD" → FC holds depth automatically
  cmd.arm = true;
  cmd.forward = static_cast<int>(effective_forward);
  cmd.lateral = static_cast<int>(lateral_pwm);
  cmd.thrust = static_cast<int>(thrust_pwm);
  cmd.yaw = static_cast<int>(yaw_pwm);
  ros_state_->cmd_publisher->publish(cmd);

  RCLCPP_INFO_THROTTLE(
      ros_state_->node->get_logger(), *ros_state_->node->get_clock(), 1000,
      "ApproachBB: Found '%s' (area=%.1f%%, x_err=%.3f, y_err=%.3f) -> Fwd: %d, Lat: %d, Thr: %d, Yaw: %d",
      target_object_.c_str(), bb_area_norm_ * 100.0, x_error, y_error,
      cmd.forward, cmd.lateral, cmd.thrust, cmd.yaw);

  return BT::NodeStatus::RUNNING;
}

void ApproachBB::onHalted() {
  RCLCPP_INFO(ros_state_->node->get_logger(), "ApproachBB: halted");
  publish_neutral();
}

void ApproachBB::publish_neutral() {
  custom_msgs::msg::Commands cmd;
  cmd.mode = flight_mode_;
  cmd.arm = true;
  cmd.forward = 1500;
  cmd.lateral = 1500;
  cmd.thrust = 1500;
  cmd.yaw = 1500;
  ros_state_->cmd_publisher->publish(cmd);
}
