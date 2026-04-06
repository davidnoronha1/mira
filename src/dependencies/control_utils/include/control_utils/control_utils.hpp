#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

class PID_Controller {
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_pub;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_callback_handle;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service;

public:
  PID_Controller(const std::string &name, rclcpp::Node::SharedPtr _node)
      : node(std::move(_node)) {

    // std::srand(static_cast<unsigned int>(std::time(nullptr))); // Seed random generator
    // int random_no = std::rand() % 1000; // For unique parameter names if multiple PID controllers are used

    const auto kp_param_name = name + "_pid_kp";
    const auto kd_param_name = name + "_pid_kd";
    const auto ki_param_name = name + "_pid_ki";
    const auto base_offset_param_name = name + "_pid_base_offset";
    node->declare_parameter<float>(kp_param_name, 0);
    node->declare_parameter<float>(kd_param_name, 0);
    node->declare_parameter<float>(ki_param_name, 0);
    node->declare_parameter<float>(base_offset_param_name, zero_offset);

    error_pub = node->create_publisher<std_msgs::msg::Float32>(
        name + "_pid_error", rclcpp::QoS(10));
    pid_pub = node->create_publisher<std_msgs::msg::Float32>(
        name + "_pid_output", rclcpp::QoS(10));

    reset_service = node->create_service<std_srvs::srv::Trigger>(
        name + "_pid_reset",
        [&](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          emptyError();
          response->success = true;
          response->message = name + " PID integral reset";
          RCLCPP_INFO(node->get_logger(), "%s PID integral reset via service call",
                      name.c_str());
        });

    kp = node->get_parameter(name + "_pid_kp").as_double();
    kd = node->get_parameter(name + "_pid_kd").as_double();
    ki = node->get_parameter(name + "_pid_ki").as_double();
    base_offset = node->get_parameter(name + "_pid_base_offset").as_double();

    param_callback_handle = node->add_post_set_parameters_callback([this, kp_param_name, kd_param_name, ki_param_name, base_offset_param_name](const std::vector<
                                             rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      std::stringstream ss;

      for (const auto &param : parameters) {
        const auto name = param.get_name();
        if (name == kp_param_name) {
          kp = param.get_value<float>();
          ss << "kp=" << kp << " ";
        } else if (name == kd_param_name) {
          kd = param.get_value<float>();
          ss << "kd=" << kd << " ";
        } else if (name == ki_param_name) {
          ki = param.get_value<float>();
          ss << "ki=" << ki << " ";
        } else if (name == base_offset_param_name) {
          base_offset = param.get_value<float>();
          ss << "base_offset=" << base_offset << " ";
        } else {
          RCLCPP_INFO(node->get_logger(), "Unknown parameter: '%s'. Only known parameters are: '%s', '%s', '%s', '%s'",
                      name.c_str(), kp_param_name.c_str(), kd_param_name.c_str(),
                      ki_param_name.c_str(), base_offset_param_name.c_str());
        }
      }

      RCLCPP_INFO(node->get_logger(), "Updated PID parameters: %s", ss.str().c_str());

      return result;
    });
  }

  auto pid_control(float error, float dtime, bool _invert = false) -> float {
    time.push_back(dtime);
    error_vector.push_back(error);

    // Prevent unbounded memory growth - keep only recent history
    if (time.size() > max_history_size) {
      time.erase(time.begin());
      error_vector.erase(error_vector.begin());
    }

    pid_p = (kp * error);
    pid_i = (integrate(error_vector, time));
    pid_d = 0;
    try {
      if (error_vector[error_vector.size() - 1] ==
          error_vector[error_vector.size() - 2]) {
        pid_d = prev_d;
      } else {
        pid_d = (kd * (error_vector[error_vector.size() - 1] -
                       error_vector[error_vector.size() - 2]));
      }
    } catch (...) {
      pid_d = 0;
    }

    pid_i = ki * pid_i;
    output_pwm = pid_p + pid_d + pid_i;
    output_pwm = base_offset + output_pwm;
    if (output_pwm > (zero_offset + safe_pwm)) {
      output_pwm = zero_offset + safe_pwm;
    } else if (output_pwm < (zero_offset - safe_pwm)) {
      output_pwm = zero_offset - safe_pwm;
    }
    prev_d = pid_d;
    pwm_prev = output_pwm;
    auto msg = std_msgs::msg::Float32();
    msg.data = output_pwm;
    pid_pub->publish(msg);
    msg.data = error;
    error_pub->publish(msg);
    return output_pwm;
  }

  static auto hold() -> float { return zero_offset; }

  void emptyError() {
    for (size_t i = 0; i < error_vector.size(); i++) {
      time.pop_back();
    }

    error_vector.clear();
  }

  float kp = 0, kd = 0, ki = 0, base_offset = zero_offset;

private:
  static constexpr float zero_offset = 1500;
  static constexpr float safe_pwm = 400;
  static constexpr size_t max_history_size = 100; // Limit history to 100 samples

  float output_pwm = 0;
  float pwm_prev = 0;
  float prev_d = 0;
  float pid_p = 0, pid_i = 0, pid_d = 0;
  std::vector<float> time;
  std::vector<float> error_vector;

  static float integrate(const std::vector<float> &error,
                         const std::vector<float> &time) {
    float area = 0;
    try {
      for (size_t i = 1; i < time.size(); i++) {
        area += (time[i] - time[i - 1]) * (error[i] + error[i - 1]) / 2;
      }
    } catch (...) {
    }
    return area;
  }
};
