#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands_normalized.hpp>
#include <utils/parameters.hpp>

namespace utils {
// Same discrete-time, conditional-integration contract as utils.control.PIDController.
class PIDController {
public:
  double kp = 0, ki = 0, kd = 0, bias = 0, limit = 1;
  void reset() { integral_ = 0; previous_ = std::numeric_limits<double>::quiet_NaN(); }
  double control(double error, double dt) {
    if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 0) { reset(); return 0; }
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        !std::isfinite(bias) || !std::isfinite(limit) || limit <= 0 || limit > 1)
      throw std::invalid_argument("Invalid normalized PID settings");
    const double derivative = std::isfinite(previous_) ? (error - previous_) / dt : 0;
    const double candidate = integral_ + error * dt;
    const double p = kp * error, d = kd * derivative;
    const double output = bias + p + ki * candidate + d;
    if (std::abs(output) <= limit || error * output < 0) integral_ = candidate;
    previous_ = error;
    return std::max(-limit, std::min(limit, bias + p + ki * integral_ + d));
  }
private:
  double integral_ = 0;
  double previous_ = std::numeric_limits<double>::quiet_NaN();
};

// Retained mission geometry uses right-positive lateral and clockwise heading.
// Convert these axes to body FLU once, without hardware/PWM knowledge.
inline void publish_mission_command(
    const rclcpp::Publisher<custom_msgs::msg::CommandsNormalized>::SharedPtr &publisher,
    const rclcpp::Node::SharedPtr &node, custom_msgs::msg::CommandsNormalized command) {
  command.header.stamp = node->now();
  command.header.frame_id = "base_link";
  command.lateral = -command.lateral;
  command.pitch = -command.pitch;
  command.yaw = -command.yaw;
  publisher->publish(command);
}
}  // namespace utils

// ROS parameter facade for existing BT leaves. Their second argument is elapsed
// mission time; the reusable core above always receives a sample interval.
class PID_Controller : public utils::PIDController {
public:
  double &base_offset = bias;
  PID_Controller(const std::string &name, rclcpp::Node::SharedPtr node) {
    auto parameter = [&](const std::string &suffix, double fallback) {
      const auto key = name + "_pid_" + suffix;
      return utils::parameter(*node, key, fallback);
    };
    kp = parameter("kp", 0); ki = parameter("ki", 0); kd = parameter("kd", 0);
    bias = parameter("base_offset", 0); limit = parameter("limit", 1);
    callback_ = node->add_post_set_parameters_callback(
        [this, name](const std::vector<rclcpp::Parameter> &parameters) {
          for (const auto &p : parameters) {
            if (p.get_name() == name + "_pid_kp") kp = p.as_double();
            else if (p.get_name() == name + "_pid_ki") ki = p.as_double();
            else if (p.get_name() == name + "_pid_kd") kd = p.as_double();
            else if (p.get_name() == name + "_pid_base_offset") bias = p.as_double();
            else if (p.get_name() == name + "_pid_limit") limit = p.as_double();
          }
        });
  }
  float pid_control(float error, float elapsed, bool invert = false) {
    const double dt = std::isfinite(last_elapsed_) ? elapsed - last_elapsed_ : 0.0;
    last_elapsed_ = elapsed;
    return static_cast<float>(control(invert ? -error : error, dt));
  }
  void emptyError() { reset(); last_elapsed_ = std::numeric_limits<double>::quiet_NaN(); }
  static float hold() { return 0; }
private:
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr callback_;
  double last_elapsed_ = std::numeric_limits<double>::quiet_NaN();
};
