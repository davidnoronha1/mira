#include <algorithm>
#include <chrono>
#include <cmath>
#include <control_utils/control_utils.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>

/*
    Forward PID Tuner (bottom camera)
    ──────────────────────────────────
    AUV is above the ArUco marker looking straight down.
    Error = marker Y position in camera frame (setpoint: 0 = centred).
    Positive Y → marker is behind → drive forward to centre it.

    Keys:
      p        Arm / start control
      q        Disarm
      w / s    Kp  +0.2 / -0.05
      e / d    Ki  +0.005 / -0.001
      r / f    Kd  +0.5 / -0.1
*/

class ForwardTunerController {
public:
  ForwardTunerController(rclcpp::Node::SharedPtr node)
      : node_(node), forward_controller_("forward", node),
        yaw_controller_("yaw", node) {

    initializePIDControllers();

    pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 1);

    keys_subscriber_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 1,
        std::bind(&ForwardTunerController::keysCallback, this,
                  std::placeholders::_1));

    aruco_subscriber_ =
        node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/aruco/error", 1,
            std::bind(&ForwardTunerController::arucoCallback, this,
                      std::placeholders::_1));

    telemetry_subscriber_ =
        node_->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 1,
            std::bind(&ForwardTunerController::telemetryCallback, this,
                      std::placeholders::_1));

    software_arm_flag_ = false;
    forward_error_ = 0.0;
    target_heading_ = -1;
    yaw_error_ = 0;

    cmd_pwm_.arm = false;
    cmd_pwm_.mode = "ALT_HOLD";
    cmd_pwm_.forward = 1500;
    cmd_pwm_.lateral = 1500;
    cmd_pwm_.thrust = 1500;
    cmd_pwm_.yaw = 1500;

    prev_time_ = node_->now();
    last_loop_time_ = node_->now();
    last_pose_time_ = node_->now();

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ForwardTunerController::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(), "[INIT] Forward tuner ready (bottom camera, Y-axis alignment)");
    printPIDParameters();
  }

private:
  void initializePIDControllers() {
    forward_controller_.kp = 200.0;
    forward_controller_.ki = 0.0;
    forward_controller_.kd = 0.0;
    forward_controller_.base_offset = 1500;

    yaw_controller_.kp = 3.18;
    yaw_controller_.ki = 0.01;
    yaw_controller_.kd = 7.2;
    yaw_controller_.base_offset = 1500;
  }

  void printPIDParameters() {
    RCLCPP_INFO(node_->get_logger(),
                "Forward PID — Kp: %.2f  Ki: %.3f  Kd: %.2f",
                forward_controller_.kp, forward_controller_.ki,
                forward_controller_.kd);
    RCLCPP_INFO(node_->get_logger(),
                "Yaw PID (heading lock) — Kp: %.2f  Ki: %.3f  Kd: %.2f",
                yaw_controller_.kp, yaw_controller_.ki, yaw_controller_.kd);
  }

  void keysCallback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;
    RCLCPP_INFO(node_->get_logger(), "[KEY] %c", key);

    switch (key) {
    case 'q':
      software_arm_flag_ = false;
      cmd_pwm_.arm = false;
      forward_controller_.emptyError();
      RCLCPP_INFO(node_->get_logger(), "[STATE] DISARMED");
      break;

    case 'p':
      software_arm_flag_ = true;
      cmd_pwm_.arm = true;
      prev_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(), "[STATE] ARMED");
      break;

    case 'w':
      forward_controller_.kp += 0.2;
      RCLCPP_INFO(node_->get_logger(), "Forward Kp: %.2f", forward_controller_.kp);
      break;
    case 's':
      forward_controller_.kp -= 0.05;
      RCLCPP_INFO(node_->get_logger(), "Forward Kp: %.2f", forward_controller_.kp);
      break;

    case 'e':
      forward_controller_.ki += 0.005;
      RCLCPP_INFO(node_->get_logger(), "Forward Ki: %.3f", forward_controller_.ki);
      break;
    case 'd':
      forward_controller_.ki -= 0.001;
      RCLCPP_INFO(node_->get_logger(), "Forward Ki: %.3f", forward_controller_.ki);
      break;

    case 'r':
      forward_controller_.kd += 0.5;
      RCLCPP_INFO(node_->get_logger(), "Forward Kd: %.2f", forward_controller_.kd);
      break;
    case 'f':
      forward_controller_.kd -= 0.1;
      RCLCPP_INFO(node_->get_logger(), "Forward Kd: %.2f", forward_controller_.kd);
      break;

    default:
      break;
    }
  }

  void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
    int heading = msg->heading;
    if (target_heading_ < 0) {
      target_heading_ = heading;
      RCLCPP_INFO(node_->get_logger(),
                  "[YAW] Heading locked at %d°", target_heading_);
    }
    // Shortest angular error [-180, 180] degrees — same as yaw_tuner.cpp
    double error = static_cast<double>(target_heading_ - heading);
    error = std::fmod(error + 180.0, 360.0);
    if (error < 0.0) error += 360.0;
    yaw_error_ = static_cast<int>(error - 180.0);
    telemetry_received_ = true;
  }

  // error.y = forward offset of marker from camera centre (metres).
  void arucoCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (!aruco_received_) {
      RCLCPP_INFO(node_->get_logger(), "[ARUCO] First error received");
      aruco_received_ = true;
    }
    forward_error_ = msg->y;
    last_pose_time_ = node_->now();
  }

  void controlLoop() {
    rclcpp::Time time_now = node_->now();

    double gap = (time_now - last_loop_time_).seconds();
    if (gap > 0.2) {
      RCLCPP_ERROR(node_->get_logger(), "[CRITICAL] Loop stalled! Gap: %.3f", gap);
    }
    last_loop_time_ = time_now;

    double dt = (time_now - prev_time_).seconds();
    prev_time_ = time_now;

    double pose_age = (time_now - last_pose_time_).seconds();
    bool pose_fresh = aruco_received_ && (pose_age < 0.5);

    if (!aruco_received_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[WARNING] No ArUco pose yet");
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[LOOP] armed=%d  forward_error(Y)=%.4f  dt=%.4f",
                         software_arm_flag_, forward_error_, dt);

    if (!telemetry_received_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[WARNING] No telemetry yet — yaw unlocked");
    }

    float pid_yaw = yaw_controller_.pid_control(yaw_error_, dt, false);
    int yaw_pwm = std::clamp(static_cast<int>(pid_yaw), 1100, 1900);

    if (software_arm_flag_ && pose_fresh) {
      cmd_pwm_.mode = "ALT_HOLD";
      cmd_pwm_.arm = true;

      float pid_forward = forward_controller_.pid_control(forward_error_, dt, false);

      cmd_pwm_.forward = std::clamp(static_cast<int>(pid_forward), 1100, 1900);
      cmd_pwm_.lateral = 1500;
      cmd_pwm_.thrust = 1500;
      cmd_pwm_.yaw = telemetry_received_ ? yaw_pwm : 1500;
    } else {
      if (software_arm_flag_ && !pose_fresh) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                             "[WARNING] Stale ArUco pose — holding neutral");
      }
      forward_controller_.emptyError();
      yaw_controller_.emptyError();
      cmd_pwm_.arm = software_arm_flag_;
      cmd_pwm_.mode = "ALT_HOLD";
      cmd_pwm_.forward = 1500;
      cmd_pwm_.lateral = 1500;
      cmd_pwm_.thrust = 1500;
      cmd_pwm_.yaw = 1500;
    }

    pwm_publisher_->publish(cmd_pwm_);
  }

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keys_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr aruco_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  PID_Controller forward_controller_;
  PID_Controller yaw_controller_;

  bool software_arm_flag_;
  bool aruco_received_ = false;
  bool telemetry_received_ = false;
  custom_msgs::msg::Commands cmd_pwm_;

  double forward_error_;
  int target_heading_;
  int yaw_error_;

  rclcpp::Time prev_time_;
  rclcpp::Time last_loop_time_;
  rclcpp::Time last_pose_time_;

  rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("forward_tuner");
  auto controller = std::make_shared<ForwardTunerController>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
