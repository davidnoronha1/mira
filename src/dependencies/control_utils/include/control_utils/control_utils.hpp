#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class PID_Controller {
public:
  float safe_pwm = 400;
  float kp, kd, ki, base_offset;
  float output_pwm;
  float pwm_prev = 0;
  float prev_d = 0;
  float pid_p, pid_i, pid_d = 0;
  std::vector<float> error_vector;

  float pid_control(float error, float dtime, bool switch_polarity) {
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
    if (output_pwm > (1500 + safe_pwm)) {
      output_pwm = 1500 + safe_pwm;
    } else if (output_pwm < (1500 - safe_pwm)) {
      output_pwm = 1500 - safe_pwm;
    }
    prev_d = pid_d;
    pwm_prev = output_pwm;
    return output_pwm;
  }

  float hold() { return 1500.00; }

  void emptyError() {
    while (!error_vector.empty()) {
      error_vector.pop_back();
      time.pop_back();
    }
  }

private:
  std::vector<float> time;

  float integrate(std::vector<float> error, std::vector<float> time) {
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
