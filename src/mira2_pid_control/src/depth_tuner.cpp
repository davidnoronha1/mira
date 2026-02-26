#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
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

class DepthTunerController {
public:
  DepthTunerController(rclcpp::Node::SharedPtr node) : node_(node), depth_controller_("depth", node), yaw_controller_("yaw", node) {

    // Initialize PID controllers
    initializePIDControllers();

    // Initialize publishers
    pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 10);

    // Initialize subscribers
    keys_subscriber_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 10, std::bind(&DepthTunerController::keysCallback, this, std::placeholders::_1));

    telemetry_subscriber_ = node_->create_subscription<custom_msgs::msg::Telemetry>(
        "/master/telemetry", 10, std::bind(&DepthTunerController::telemetryCallback, this, std::placeholders::_1));

    // Initialize control parameters
    software_arm_flag_ = false;
    depth_setpoint_ = 1100.0;
    yaw_setpoint_ = 90;
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
    init_time_ = node_->now();
    start_routine_ = node_->now();

    // Create timer for control loop (10 Hz)
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DepthTunerController::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(), "Depth Tuner Controller initialized");
    printPIDParameters();
  }

private:
  void initializePIDControllers() {
    // Depth PID initialization
    depth_controller_.kp = 5.8;
    depth_controller_.ki = 0.0;
    depth_controller_.kd = 20.0;
    depth_controller_.base_offset = 1580;

    // Yaw PID initialization
    yaw_controller_.kp = 0.0;
    yaw_controller_.ki = 0.0;
    yaw_controller_.kd = 0.0;
    yaw_controller_.base_offset = 1500;
  }

  void printPIDParameters() {
    RCLCPP_INFO(node_->get_logger(), "Depth PID - Kp: %.2f, Ki: %.3f, Kd: %.2f",
                depth_controller_.kp, depth_controller_.ki, depth_controller_.kd);
    RCLCPP_INFO(node_->get_logger(), "Yaw PID - Kp: %.2f, Ki: %.3f, Kd: %.2f",
                yaw_controller_.kp, yaw_controller_.ki, yaw_controller_.kd);
  }

  void keysCallback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;

    switch (key) {
      case 'q':
        software_arm_flag_ = false;
        RCLCPP_INFO(node_->get_logger(), "UNARMED");
        start_routine_ = node_->now();
        depth_controller_.emptyError();
        break;

      case 'p':
        software_arm_flag_ = true;
        RCLCPP_INFO(node_->get_logger(), "ARMED");
        start_routine_ = node_->now();
        break;

      case 'w':
        depth_controller_.kp += 0.2;
        RCLCPP_INFO(node_->get_logger(), "Depth Kp: %.2f", depth_controller_.kp);
        break;

      case 's':
        depth_controller_.kp -= 0.05;
        RCLCPP_INFO(node_->get_logger(), "Depth Kp: %.2f", depth_controller_.kp);
        break;

      case 'e':
        depth_controller_.ki += 0.005;
        RCLCPP_INFO(node_->get_logger(), "Depth Ki: %.3f", depth_controller_.ki);
        break;

      case 'd':
        depth_controller_.ki -= 0.001;
        RCLCPP_INFO(node_->get_logger(), "Depth Ki: %.3f", depth_controller_.ki);
        break;

      case 'r':
        depth_controller_.kd += 0.1;
        RCLCPP_INFO(node_->get_logger(), "Depth Kd: %.2f", depth_controller_.kd);
        break;

      case 'f':
        depth_controller_.kd -= 0.1;
        RCLCPP_INFO(node_->get_logger(), "Depth Kd: %.2f", depth_controller_.kd);
        break;

      default:
        break;
    }
  }

  void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
    double depth_external = msg->external_pressure;
    int yaw_heading = msg->heading;

    depth_error_ = depth_external - depth_setpoint_;
    yaw_error_ = yaw_setpoint_ - yaw_heading;
  }

  void controlLoop() {
    if (software_arm_flag_) {
      cmd_pwm_.mode = "STABILIZE";
      cmd_pwm_.arm = true;

      rclcpp::Time time_now = node_->now();
      double dt = (time_now - init_time_).seconds();

      float pid_depth = depth_controller_.pid_control(depth_error_, dt, false);
      float pid_yaw = yaw_controller_.pid_control(yaw_error_, dt, false);

      RCLCPP_DEBUG(node_->get_logger(), "PID Depth: %.2f, PID Yaw: %.2f", pid_depth, pid_yaw);

      cmd_pwm_.forward = 1500;
      cmd_pwm_.lateral = 1500;
      cmd_pwm_.thrust = static_cast<int>(pid_depth);
      cmd_pwm_.yaw = 1500;

      RCLCPP_DEBUG(node_->get_logger(), "Sinking - Thrust: %d", cmd_pwm_.thrust);
    } else {
      depth_controller_.emptyError();

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
  double depth_setpoint_;
  double depth_error_;
  int yaw_setpoint_;
  int yaw_error_;

  // Timing
  rclcpp::Time init_time_;
  rclcpp::Time start_routine_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("depth_tuner_controller");
  auto controller = std::make_shared<DepthTunerController>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
