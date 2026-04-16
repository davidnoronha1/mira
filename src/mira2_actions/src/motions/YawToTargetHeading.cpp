#include "motions.hpp"

RotateToTargetHeading::RotateToTargetHeading(const std::string &name,
                      const BT::NodeConfiguration &config,
                      ROSState *ros_state)
    : BT::SyncActionNode(name, config), 
      yaw_pid(name + "_yaw", ros_state->node),
      ros_state_(ros_state),
      target_heading_(0.0),
      yaw_error_(0.0),
      initialized_(false) 
{}

BT::PortsList RotateToTargetHeading::providedPorts() {
    return {
        BT::InputPort<double>("target_heading",
                              "Desired heading to rotate to (degrees)"),
        BT::InputPort<double>("yaw_pid_kp", 3.18, "Yaw PID proportional gain"),
        BT::InputPort<double>("yaw_pid_ki", 0.01, "Yaw PID integral gain"),
        BT::InputPort<double>("yaw_pid_kd", 7.2, "Yaw PID derivative gain"),
        BT::InputPort<double>("yaw_pid_base_offset", 1500.0,
                              "Yaw PID base offset"),
        BT::InputPort<double>("tolerance", 5.0, "Heading tolerance in degrees"),
        BT::InputPort<double>("timeout", 30.0,
                              "Maximum time to attempt rotation (seconds)")};
  }

BT::NodeStatus RotateToTargetHeading::tick() {
    // Initialize on first tick
    if (!initialized_) {
      // Get target heading from port
      auto target = getInput<double>("target_heading");
      if (!target) {
        throw BT::RuntimeError("Missing required input [target_heading]: ",
                               target.error());
      }
      target_heading_ = target.value();

      // Get PID parameters from ports
      auto kp = getInput<double>("yaw_pid_kp").value();
      auto ki = getInput<double>("yaw_pid_ki").value();
      auto kd = getInput<double>("yaw_pid_kd").value();
      auto base_offset = getInput<double>("yaw_pid_base_offset").value();

      // Configure PID controller
      yaw_pid.kp = kp;
      yaw_pid.ki = ki;
      yaw_pid.kd = kd;
      yaw_pid.base_offset = base_offset;
      yaw_pid.emptyError();

      start_time_ = ros_state_->node->now();
      initialized_ = true;

      RCLCPP_INFO(ros_state_->node->get_logger(),
                  "RotateToTargetHeading: Starting rotation to %.1f degrees",
                  target_heading_);
    }

    // Check for timeout
    auto timeout = getInput<double>("timeout").value();
    double elapsed = (ros_state_->node->now() - start_time_).seconds();
    if (elapsed > timeout) {
      RCLCPP_WARN(ros_state_->node->get_logger(),
                  "RotateToTargetHeading: Timeout reached after %.1f seconds",
                  elapsed);
      initialized_ = false;
      return BT::NodeStatus::FAILURE;
    }

    // Calculate yaw error (shortest angle difference)
    // NISHANNTH: CHECK THIS !!!
    double current_heading = ros_state_->telemetry.heading;
    yaw_error_ =
        std::fmod((target_heading_ - current_heading + 180.0), 360.0) - 180.0;

    // Check if we've reached the target
    auto tolerance = getInput<double>("tolerance").value();
    if (std::abs(yaw_error_) < tolerance) {
      RCLCPP_INFO(
          ros_state_->node->get_logger(),
          "RotateToTargetHeading: Target heading reached (error: %.2f deg)",
          yaw_error_);

      // Send neutral commands before finishing
      custom_msgs::msg::Commands cmd;
      cmd.mode = "STABILIZE";
      cmd.arm = true;
      cmd.forward = 1500;
      cmd.lateral = 1500;
      cmd.thrust = 1500;
      cmd.yaw = 1500;
      ros_state_->cmd_publisher->publish(cmd);

      initialized_ = false;
      return BT::NodeStatus::SUCCESS;
    }

    // Calculate PID output for yaw
    float pid_yaw = yaw_pid.pid_control(yaw_error_, elapsed, false);

    // Publish command
    custom_msgs::msg::Commands cmd;
    cmd.mode = "STABILIZE";
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust = 1500; // Hold depth (could add depth PID here if needed)
    cmd.yaw = static_cast<int>(pid_yaw);
    ros_state_->cmd_publisher->publish(cmd);

    RCLCPP_DEBUG(ros_state_->node->get_logger(),
                 "RotateToTargetHeading: Current: %.1f, Target: %.1f, Error: "
                 "%.2f, YawPWM: %d",
                 current_heading, target_heading_, yaw_error_, cmd.yaw);

    return BT::NodeStatus::RUNNING;
  }
