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
#define THRESHOLD 8 // degrees

class YawTunerController {
public:
  YawTunerController(rclcpp::Node::SharedPtr node) 
      : node_(node), 
        depth_controller_("depth", node), 
        yaw_controller_("yaw", node) {
    
    // Initialize PID controllers
    initializePIDControllers();
    
    // Publishers
    pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 1);

    // Subscribers
    keys_subscriber_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 1,
        std::bind(&YawTunerController::keysCallback, this, std::placeholders::_1));
    
    telemetry_subscriber_ = node_->create_subscription<custom_msgs::msg::Telemetry>(
        "/master/telemetry", 1,
        std::bind(&YawTunerController::telemetryCallback, this, std::placeholders::_1));

    // Initialize control parameters
    software_arm_flag_ = false;
    yaw_setpoint_ = 200;
    depth_error_ = 0.0;
    yaw_error_ = 0;

    // Initialize command message
    cmd_pwm_.arm = false;
    cmd_pwm_.mode = "STABILIZE";
    cmd_pwm_.forward = 1500;
    cmd_pwm_.lateral = 1500;
    cmd_pwm_.thrust = 1500;
    cmd_pwm_.yaw = 1500;

    // Initialize timing
    start_routine_ = node_->now();

    // Timer for control loop
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&YawTunerController::controlLoop, this));
    
    RCLCPP_INFO(node_->get_logger(), "Yaw Tuner Controller initialized");
    printPIDParameters();
  }

private:
  void initializePIDControllers() {
    // Yaw PID initialization
    yaw_controller_.kp = 3.18;
    yaw_controller_.ki = 0.01;
    yaw_controller_.kd = 7.2;
    yaw_controller_.base_offset = 1500;

    // Depth PID (not actively used but initialized)
    depth_controller_.kp = 0.0;
    depth_controller_.ki = 0.0;
    depth_controller_.kd = 0.0;
    depth_controller_.base_offset = 1500;
  }

  void printPIDParameters() {
    RCLCPP_INFO(node_->get_logger(), "Yaw PID - Kp: %.2f, Ki: %.3f, Kd: %.2f",
                yaw_controller_.kp, yaw_controller_.ki, yaw_controller_.kd);
  }

  void keysCallback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;
    
    switch (key) {
      case 'q':
        software_arm_flag_ = false;
        cmd_pwm_.arm = false;
        RCLCPP_INFO(node_->get_logger(), "UNARMED");
        start_routine_ = node_->now();
        depth_controller_.emptyError();
        yaw_controller_.emptyError();
        break;
        
      case 'p':
        software_arm_flag_ = true;
        cmd_pwm_.arm = true;
        RCLCPP_INFO(node_->get_logger(), "ARMED");
        start_routine_ = node_->now();
        break;
        
      case 'w':
        yaw_controller_.kp += 0.2;
        RCLCPP_INFO(node_->get_logger(), "Yaw Kp: %.2f", yaw_controller_.kp);
        break;
        
      case 's':
        yaw_controller_.kp -= 0.05;
        RCLCPP_INFO(node_->get_logger(), "Yaw Kp: %.2f", yaw_controller_.kp);
        break;
        
      case 'e':
        yaw_controller_.ki += 0.005;
        RCLCPP_INFO(node_->get_logger(), "Yaw Ki: %.3f", yaw_controller_.ki);
        break;
        
      case 'd':
        yaw_controller_.ki -= 0.001;
        RCLCPP_INFO(node_->get_logger(), "Yaw Ki: %.3f", yaw_controller_.ki);
        break;
        
      case 'r':
        yaw_controller_.kd += 0.1;
        RCLCPP_INFO(node_->get_logger(), "Yaw Kd: %.2f", yaw_controller_.kd);
        break;
        
      case 'f':
        yaw_controller_.kd -= 0.1;
        RCLCPP_INFO(node_->get_logger(), "Yaw Kd: %.2f", yaw_controller_.kd);
        break;
        
      case 'j':
        yaw_setpoint_ += 45;
        if (yaw_setpoint_ > 360) {
          yaw_setpoint_ = 0;
        }
        RCLCPP_INFO(node_->get_logger(), "Yaw setpoint: %d", yaw_setpoint_);
        break;
        
      default:
        break;
    }
  }

  void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
    double depth_external = msg->external_pressure;
    int yaw_heading = msg->heading;

    // Shortest-angle difference in [-180, 180]
    yaw_error_ = static_cast<int>(
        std::fmod((yaw_setpoint_ - yaw_heading + 180.0), 360.0) - 180.0);
  }

  void controlLoop() {
    if (software_arm_flag_) {
      // Use ALT_HOLD for depth-hold behavior
      cmd_pwm_.mode = "ALT_HOLD";
      rclcpp::Time time_now = node_->now();
      cmd_pwm_.arm = true;

      double elapsed = (time_now - start_routine_).seconds();

      // FIRST PHASE: fixed downward thrust for 2.8 seconds
      if (elapsed < 2.8) {
        cmd_pwm_.forward = 1500;
        cmd_pwm_.lateral = 1500;
        cmd_pwm_.thrust = 1450; // fixed descent thrust
        cmd_pwm_.yaw = 1500;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Sinking for initial 2.8s (thrust=1450), elapsed: %.2f", 
                            elapsed);
      } else {
        // SECOND PHASE: start yaw tuning
        float pid_yaw = yaw_controller_.pid_control(
            yaw_error_, (time_now - start_routine_).seconds(), false);

        cmd_pwm_.forward = 1500;
        cmd_pwm_.lateral = 1500;
        cmd_pwm_.thrust = 1500; // allow depth hold to maintain depth
        cmd_pwm_.yaw = static_cast<int>(pid_yaw);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Tuning yaw (elapsed: %.2f). pid_yaw: %.2f", 
                            elapsed, pid_yaw);
      }
    } else {
      depth_controller_.emptyError();
      yaw_controller_.emptyError();
      cmd_pwm_.arm = false;
      cmd_pwm_.mode = "STABILIZE";
      cmd_pwm_.forward = 1500;
      cmd_pwm_.lateral = 1500;
      cmd_pwm_.thrust = 1500;
      cmd_pwm_.yaw = 1500;
    }
    
    pwm_publisher_->publish(cmd_pwm_);
  }

  // ROS2 Node
  rclcpp::Node::SharedPtr node_;

  // ROS2 Publishers and Subscribers
  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keys_subscriber_;
  rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // PID Controllers
  PID_Controller depth_controller_;
  PID_Controller yaw_controller_;

  // Control state
  bool software_arm_flag_;
  custom_msgs::msg::Commands cmd_pwm_;

  // Setpoints and errors
  double depth_error_;
  int yaw_setpoint_;
  int yaw_error_;

  // Timing
  rclcpp::Time start_routine_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yaw_tuner_controller");
  auto controller = std::make_shared<YawTunerController>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}