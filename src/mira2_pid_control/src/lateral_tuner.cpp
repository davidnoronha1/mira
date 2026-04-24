#include <algorithm>
#include <chrono>
#include <control_utils/control_utils.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>

/*
    Lateral PID Tuner
    ─────────────────
    Subscribes to /aruco/error (geometry_msgs/Vector3) published by aruco_tracker.
    error.x = lateral offset of marker from camera centre (metres).
    Setpoint: 0. Positive x → marker right of centre → drive lateral to centre.

    Keys:
      p        Arm / start control
      q        Disarm
      w / s    Kp  +0.2 / -0.05
      e / d    Ki  +0.005 / -0.001
      r / f    Kd  +0.5 / -0.1
*/

class LateralTunerController {
public:
  LateralTunerController(rclcpp::Node::SharedPtr node)
      : node_(node), lateral_controller_("lateral", node) {

    initializePIDControllers();

    pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 1);

    keys_subscriber_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 1,
        std::bind(&LateralTunerController::keysCallback, this,
                  std::placeholders::_1));

    aruco_subscriber_ =
        node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/aruco/error", 1,
            std::bind(&LateralTunerController::arucoCallback, this,
                      std::placeholders::_1));

    software_arm_flag_ = false;
    lateral_error_ = 0.0;

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
        std::bind(&LateralTunerController::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(), "[INIT] Lateral tuner ready");
    printPIDParameters();
  }

private:
  void initializePIDControllers() {
    lateral_controller_.kp = 200.0;
    lateral_controller_.ki = 0.0;
    lateral_controller_.kd = 0.0;
    lateral_controller_.base_offset = 1500;
  }

  void printPIDParameters() {
    RCLCPP_INFO(node_->get_logger(),
                "Lateral PID — Kp: %.2f  Ki: %.3f  Kd: %.2f",
                lateral_controller_.kp, lateral_controller_.ki,
                lateral_controller_.kd);
  }

  void keysCallback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;
    RCLCPP_INFO(node_->get_logger(), "[KEY] %c", key);

    switch (key) {
    case 'q':
      software_arm_flag_ = false;
      cmd_pwm_.arm = false;
      lateral_controller_.emptyError();
      RCLCPP_INFO(node_->get_logger(), "[STATE] DISARMED");
      break;

    case 'p':
      software_arm_flag_ = true;
      cmd_pwm_.arm = true;
      prev_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(), "[STATE] ARMED");
      break;

    case 'w':
      lateral_controller_.kp += 0.2;
      RCLCPP_INFO(node_->get_logger(), "Lateral Kp: %.2f",
                  lateral_controller_.kp);
      break;
    case 's':
      lateral_controller_.kp -= 0.05;
      RCLCPP_INFO(node_->get_logger(), "Lateral Kp: %.2f",
                  lateral_controller_.kp);
      break;

    case 'e':
      lateral_controller_.ki += 0.005;
      RCLCPP_INFO(node_->get_logger(), "Lateral Ki: %.3f",
                  lateral_controller_.ki);
      break;
    case 'd':
      lateral_controller_.ki -= 0.001;
      RCLCPP_INFO(node_->get_logger(), "Lateral Ki: %.3f",
                  lateral_controller_.ki);
      break;

    case 'r':
      lateral_controller_.kd += 0.5;
      RCLCPP_INFO(node_->get_logger(), "Lateral Kd: %.2f",
                  lateral_controller_.kd);
      break;
    case 'f':
      lateral_controller_.kd -= 0.1;
      RCLCPP_INFO(node_->get_logger(), "Lateral Kd: %.2f",
                  lateral_controller_.kd);
      break;

    default:
      break;
    }
  }

  // error.x = lateral offset of marker from camera centre (metres).
  void arucoCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (!aruco_received_) {
      RCLCPP_INFO(node_->get_logger(), "[ARUCO] First error received");
      aruco_received_ = true;
    }
    lateral_error_ = msg->x;
    last_pose_time_ = node_->now();
  }

  void controlLoop() {
    rclcpp::Time time_now = node_->now();

    double gap = (time_now - last_loop_time_).seconds();
    if (gap > 0.2) {
      RCLCPP_ERROR(node_->get_logger(), "[CRITICAL] Loop stalled! Gap: %.3f",
                   gap);
    }
    last_loop_time_ = time_now;

    double dt = (time_now - prev_time_).seconds();
    prev_time_ = time_now;

    // Stale pose guard: if no ArUco for >0.5 s, hold neutral.
    double pose_age = (time_now - last_pose_time_).seconds();
    bool pose_fresh = aruco_received_ && (pose_age < 0.5);

    if (!aruco_received_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[WARNING] No ArUco pose yet");
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[LOOP] armed=%d  lateral_error=%.4f  dt=%.4f",
                         software_arm_flag_, lateral_error_, dt);

    if (software_arm_flag_ && pose_fresh) {
      cmd_pwm_.mode = "ALT_HOLD";
      cmd_pwm_.arm = true;

      float pid_lateral =
          lateral_controller_.pid_control(lateral_error_, dt, false);

      cmd_pwm_.forward = 1500;
      cmd_pwm_.lateral = std::clamp(static_cast<int>(pid_lateral), 1100, 1900);
      cmd_pwm_.thrust = 1500;
      cmd_pwm_.yaw = 1500;
    } else {
      if (software_arm_flag_ && !pose_fresh) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                             "[WARNING] Stale ArUco pose — holding neutral");
      }
      lateral_controller_.emptyError();
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

  PID_Controller lateral_controller_;

  bool software_arm_flag_;
  bool aruco_received_ = false;
  custom_msgs::msg::Commands cmd_pwm_;

  double lateral_error_;

  rclcpp::Time prev_time_;
  rclcpp::Time last_loop_time_;
  rclcpp::Time last_pose_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lateral_tuner");
  auto controller = std::make_shared<LateralTunerController>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
