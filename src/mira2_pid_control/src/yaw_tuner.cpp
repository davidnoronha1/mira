#include <algorithm>
#include <cmath>
#include <control_utils/control_utils.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// Threshold (currently unused, placeholder for future logic)
#define THRESHOLD 8 // degrees

class YawTunerController {
public:
  // Constructor: sets up node, controllers, communication, and timers
  YawTunerController(rclcpp::Node::SharedPtr node)
      : node_(node), depth_controller_("depth", node),
        yaw_controller_("yaw", node) {

    RCLCPP_INFO(node_->get_logger(), "[INIT] Starting controller");

    // Initialize PID gains
    initializePIDControllers();
    RCLCPP_INFO(node_->get_logger(), "[INIT] PID initialized");

    // Publisher: sends PWM commands to vehicle
    pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 1);
    RCLCPP_INFO(node_->get_logger(), "[INIT] Publisher created");

    // Subscriber: keyboard input for live tuning
    keys_subscriber_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 1,
        std::bind(&YawTunerController::keysCallback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "[INIT] Key subscriber ready");

    // Subscriber: telemetry (heading + depth)
    telemetry_subscriber_ =
        node_->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 1,
            std::bind(&YawTunerController::telemetryCallback, this,
                      std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "[INIT] Telemetry subscriber ready");

    // Initial system state
    software_arm_flag_ = false;
    yaw_setpoint_ = 200;
    depth_error_ = 0.0;
    yaw_error_ = 0;

    // Neutral PWM initialization
    cmd_pwm_.arm = false;
    cmd_pwm_.mode = "ALT_HOLD";
    cmd_pwm_.forward = 1500;
    cmd_pwm_.lateral = 1500;
    cmd_pwm_.thrust = 1500;
    cmd_pwm_.yaw = 1500;

    // Timing variables
    start_routine_ = node_->now();
    prev_time_ = node_->now();
    last_loop_time_ = node_->now();

    // Control loop at 100 Hz
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&YawTunerController::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(), "[INIT] Control loop started");
    RCLCPP_INFO(node_->get_logger(), "[INIT] Initialization COMPLETE");

    printPIDParameters();
  }

private:
  // Initialize PID controller parameters
  void initializePIDControllers() {
    yaw_controller_.kp = 3.18;
    yaw_controller_.ki = 0.01;
    yaw_controller_.kd = 7.2;
    yaw_controller_.base_offset = 1500;
    // Depth PID currently unused
    depth_controller_.kp = 0.0;
    depth_controller_.ki = 0.0;
    depth_controller_.kd = 0.0;
    depth_controller_.base_offset = 1500;
  }

  // Print PID values to terminal
  void printPIDParameters() {
    RCLCPP_INFO(node_->get_logger(), "Yaw PID - Kp: %.2f, Ki: %.3f, Kd: %.2f",
                yaw_controller_.kp, yaw_controller_.ki, yaw_controller_.kd);
  }

  // Keyboard callback: used for arming + PID tuning
  void keysCallback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;

    // Debug: log every key press
    RCLCPP_INFO(node_->get_logger(), "[KEY] Pressed: %c", key);

    switch (key) {
    case 'q': // Disarm system
      software_arm_flag_ = false;
      cmd_pwm_.arm = false;
      RCLCPP_INFO(node_->get_logger(), "[STATE] UNARMED");

      // Reset timers and PID memory
      start_routine_ = node_->now();
      yaw_controller_.emptyError();
      break;

    case 'p': // Arm system
      software_arm_flag_ = true;
      cmd_pwm_.arm = true;
      RCLCPP_INFO(node_->get_logger(), "[STATE] ARMED");

      start_routine_ = node_->now();
      prev_time_ = node_->now();
      break;

    case 'j': // Increment yaw setpoint
      yaw_setpoint_ = (yaw_setpoint_ + 45) % 360;
      RCLCPP_INFO(node_->get_logger(), "[SETPOINT] Yaw: %d", yaw_setpoint_);
      break;

    default:
      break;
    }
  }

  // Telemetry callback: updates yaw error from sensor data
  void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {

    // Debug: confirm telemetry is being received
    if (!telemetry_received_) {
      RCLCPP_INFO(node_->get_logger(), "[TELEMETRY] First message received");
      telemetry_received_ = true;
    }

    double depth_external = msg->external_pressure; // currently unused
    int yaw_heading = msg->heading;

    // Compute shortest angular error [-180, 180]
    double error = yaw_setpoint_ - yaw_heading;
    error = std::fmod(error + 180.0, 360.0);
    if (error < 0)
      error += 360.0;
    yaw_error_ = static_cast<int>(error - 180.0);
  }

  // Main control loop (runs every 10 ms)
  void controlLoop() {
    rclcpp::Time time_now = node_->now();

    // Detect if loop stalls (timing gap too large)
    double gap = (time_now - last_loop_time_).seconds();
    if (gap > 0.2) {
      RCLCPP_ERROR(node_->get_logger(), "[CRITICAL] Loop stalled! Gap: %.3f",
                   gap);
    }
    last_loop_time_ = time_now;

    // First loop indicator
    if (first_loop_) {
      RCLCPP_INFO(node_->get_logger(), "[LOOP] First execution");
      first_loop_ = false;
    }

    // Compute delta time for PID
    double dt = (time_now - prev_time_).seconds();
    prev_time_ = time_now;

    // Warn if timing becomes unstable
    if (dt > 0.1) {
      RCLCPP_WARN(node_->get_logger(), "[LOOP WARNING] Large dt: %.3f", dt);
    }

    // Warn if telemetry is missing
    if (!telemetry_received_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[WARNING] No telemetry yet");
    }

    // Heartbeat log every 2 seconds
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[LOOP] Running | yaw_error: %d | dt: %.4f",
                         yaw_error_, dt);

    if (software_arm_flag_) {
      cmd_pwm_.mode = "ALT_HOLD";
      cmd_pwm_.arm = true;

      double elapsed = (time_now - start_routine_).seconds();

      // Phase 1: initial descent
      if (elapsed < 2.8) {
        cmd_pwm_.forward = 1500;
        cmd_pwm_.lateral = 1500;
        cmd_pwm_.thrust = 1500;
        cmd_pwm_.yaw = 1500;

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[PHASE] Sinking | t: %.2f", elapsed);
      }
      // Phase 2: yaw control using PID
      else {
        float pid_yaw = yaw_controller_.pid_control(yaw_error_, dt, false);

        cmd_pwm_.forward = 1500;
        cmd_pwm_.lateral = 1500;
        cmd_pwm_.thrust = 1500;

        // Clamp output to safe PWM range
        cmd_pwm_.yaw = std::clamp(static_cast<int>(pid_yaw), 1100, 1900);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[PHASE] Yaw control | pid: %.2f", pid_yaw);
      }
    } else {
      // Reset everything when disarmed
      depth_controller_.emptyError();
      yaw_controller_.emptyError();

      cmd_pwm_.arm = false;
      cmd_pwm_.mode = "ALT_HOLD";
      cmd_pwm_.forward = 1500;
      cmd_pwm_.lateral = 1500;
      cmd_pwm_.thrust = 1500;
      cmd_pwm_.yaw = 1500;
    }

    // Publish commands every cycle
    pwm_publisher_->publish(cmd_pwm_);
  }

  // ROS Node
  rclcpp::Node::SharedPtr node_;

  // Communication interfaces
  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keys_subscriber_;
  rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr
      telemetry_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // PID controllers
  PID_Controller depth_controller_;
  PID_Controller yaw_controller_;

  // System state
  bool software_arm_flag_;
  bool telemetry_received_ = false;
  bool first_loop_ = true;

  custom_msgs::msg::Commands cmd_pwm_;

  // Control variables
  double depth_error_; // unused but retained
  int yaw_setpoint_;
  int yaw_error_;

  // Timing variables
  rclcpp::Time start_routine_;
  rclcpp::Time prev_time_;
  rclcpp::Time last_loop_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("yaw_tuner_controller");
  auto controller = std::make_shared<YawTunerController>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
