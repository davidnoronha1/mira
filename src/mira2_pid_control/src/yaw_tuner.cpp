#include <cmath>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <iostream>
#include <control_utils/control_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
/*
    Bot Orientation
    +ve   x  -ve (CCW)
    (CW)  |
    y -- bot
    +ve
*/

// Predefined Variables
#define threshold 8 // degrees

// Control parameters and PWM Commands
bool software_arm_flag = false;
custom_msgs::msg::Commands cmd_pwm;
PID_Controller depth, yaw;
double depth_error, depth_setpoint;
int yaw_error, yaw_setpoint;
rclcpp::Time start_routine;

class YawTuner : public rclcpp::Node {
public:
  YawTuner() : Node("yaw_tuner_controller") {
    // Publishers
    pwm_publisher_ = this->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 1);

    // Subscribers
    keys_subscriber_ = this->create_subscription<std_msgs::msg::Char>(
        "keys", 1,
        std::bind(&YawTuner::keys_callback, this, std::placeholders::_1));
    telemetry_subscriber_ =
        this->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 1,
            std::bind(&YawTuner::telemetry_callback, this,
                      std::placeholders::_1));

   
    yaw.kp = 3.18;
    yaw.ki = 0.01;
    yaw.kd = 7.2;
    yaw.base_offset = 1500;
    yaw_setpoint = 200;

    cmd_pwm.arm = false;

    // Timer for control loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&YawTuner::control_loop, this));
  }

private:
  void keys_callback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;
    if (key == 'q') {
      software_arm_flag = false;
      cmd_pwm.arm = false;
      RCLCPP_INFO(this->get_logger(), "unarmed");
      start_routine = this->now();
      depth.emptyError();
      yaw.emptyError();
    } else if (key == 'p') {
      software_arm_flag = true;
      cmd_pwm.arm = true;
      RCLCPP_INFO(this->get_logger(), "armed");
      start_routine = this->now();
    } else if (key == 'w') {
      yaw.kp += 0.2;
      RCLCPP_INFO(this->get_logger(), "current yaw kp value: %f", yaw.kp);
    } else if (key == 's') {
      yaw.kp -= 0.05;
      RCLCPP_INFO(this->get_logger(), "current yaw kp value: %f", yaw.kp);
    } else if (key == 'e') {
      yaw.ki += 0.005;
      RCLCPP_INFO(this->get_logger(), "current yaw ki value: %f", yaw.ki);
    } else if (key == 'd') {
      yaw.ki -= 0.001;
      RCLCPP_INFO(this->get_logger(), "current yaw ki value: %f", yaw.ki);
    } else if (key == 'r') {
      yaw.kd += 0.1;
      RCLCPP_INFO(this->get_logger(), "current yaw kd value: %f", yaw.kd);
    } else if (key == 'f') {
      yaw.kd -= 0.1;
      RCLCPP_INFO(this->get_logger(), "current yaw kd value: %f", yaw.kd);
    } else if (key == 'j') {
      yaw_setpoint += 45;
      if (yaw_setpoint > 360) {
        yaw_setpoint = 0;
      }
      RCLCPP_INFO(this->get_logger(), "Current yaw setpoint is %d",
                  yaw_setpoint);
    }
  }

  void telemetry_callback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
    double depth_external = msg->external_pressure;
    int yaw_heading = msg->heading;

    // shortest-angle difference in [-180, 180]
    yaw_error = static_cast<int>(std::fmod((yaw_setpoint - yaw_heading + 180.0), 360.0) - 180.0);
  }

  void control_loop() {
    if (software_arm_flag) {
      // Use ALT_HOLD for depth-hold behavior
      cmd_pwm.mode = "ALT_HOLD";
      rclcpp::Time time_now = this->now();
      cmd_pwm.arm = true;

      double elapsed = (time_now - start_routine).seconds();

      // FIRST PHASE: fixed downward thrust for 2.8 seconds
      if (elapsed < 2.8) {
        // fixed sink command (1450) for 2.8 seconds
        cmd_pwm.forward = 1500;
        cmd_pwm.lateral = 1500;
        cmd_pwm.thrust = 1450; // fixed descent thrust
        cmd_pwm.yaw = 1500;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sinking for initial 2.8s (thrust=1450), elapsed: %.2f", elapsed);
      } else {
        // SECOND PHASE: start yaw tuning — keep ALT_HOLD and run yaw PID
        // depth PID can still run to provide thrust corrections while yaw PID controls yaw
        float pid_yaw = yaw.pid_control(yaw_error, (time_now - start_routine).seconds(), false);

        cmd_pwm.forward = 1500;
        cmd_pwm.lateral = 1500;
        cmd_pwm.thrust = 1500; // allow depth PID to maintain depth
        cmd_pwm.yaw = pid_yaw;      // yaw PID output controls yaw PWM
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Now tuning yaw (elapsed: %.2f). pid_yaw: %.2f, ", elapsed, (double)pid_yaw);
      }
    } else {
      depth.emptyError();
      yaw.emptyError();
      cmd_pwm.arm = false;
      cmd_pwm.mode = "STABILIZE";
      cmd_pwm.forward = 1500;
      cmd_pwm.lateral = 1500;
      cmd_pwm.thrust = 1500;
      cmd_pwm.yaw = 1500;
    }
    pwm_publisher_->publish(cmd_pwm);
  }

  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keys_subscriber_;
  rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr
      telemetry_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawTuner>());
  rclcpp::shutdown();
  return 0;
}