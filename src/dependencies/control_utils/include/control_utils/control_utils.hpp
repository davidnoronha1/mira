#include <cmath>
#include <exception>
#include <geometry_msgs/msg/vector3.hpp>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

class PID_Controller {
  rclcpp::Node::SharedPtr node;

public:
  PID_Controller(const std::string& name, rclcpp::Node::SharedPtr _node)
      : node(_node) {
    node->declare_parameter<float>(name + "_pid_kp", 0);
    node->declare_parameter<float>(name + "_pid_kd", 0);
    node->declare_parameter<float>(name + "_pid_ki", 0);
    node->declare_parameter<float>(name + "_pid_base_offset", zero_offset);

    node->add_on_set_parameters_callback(
        [&](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;

          for (const auto &param : parameters) {
            if (param.get_name() == (name + "_pid_kp")) {
              kp = param.get_value<float>();
            } else if (param.get_name() == (name + "_pid_kd")) {
              kd = param.get_value<float>();
            } else if (param.get_name() == (name + "_pid_ki")) {
              ki = param.get_value<float>();
            } else if (param.get_name() == (name + "_pid_base_offset")) {
              base_offset = param.get_value<float>();
            }
          }

          RCLCPP_INFO(node->get_logger(), "Updated PID parameters: kp=%.2f, kd=%.2f, ki=%.2f, base_offset=%.2f",
                       kp, kd, ki, base_offset);
          return result;
        });

      kp = node->get_parameter(name + "_pid_kp").as_double();
      kd = node->get_parameter(name + "_pid_kd").as_double();
      ki = node->get_parameter(name + "_pid_ki").as_double();
      base_offset = node->get_parameter(name + "_pid_base_offset").as_double();
  }

  auto pid_control(float error, float dtime, bool _invert = false) -> float {
    time.push_back(dtime);
    error_vector.push_back(error);
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

  float output_pwm;
  float pwm_prev = 0;
  float prev_d = 0;
  float pid_p, pid_i, pid_d = 0;
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
