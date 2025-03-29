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
#define threshold 8 // degrees

// Control parameters and PWM Commands
bool software_arm_flag = false;
custom_msgs::msg::Commands cmd_pwm;
PID_Controller depth, yaw;
double depth_error, depth_setpoint;
int yaw_error, yaw_setpoint;
rclcpp::Time start_routine;

/* Keys callback
    Function for tuning the PID parameters
*/
void keys_callback(const std_msgs::msg::Char::SharedPtr msg) {
  char key = msg->data;
  if (key == 'q') {
    software_arm_flag = false;
    std::cout << "unarmed\n";
    start_routine = rclcpp::Clock().now();
    depth.emptyError();
  } else if (key == 'p') {
    software_arm_flag = true;
    std::cout << "armed\n";
    start_routine = rclcpp::Clock().now();
  } else if (key == 'w') {
    depth.kp = depth.kp + 0.2;
    std::cout << "current depth kp value: " + std::to_string(depth.kp)
              << std::endl;
  } else if (key == 's') {
    depth.kp = depth.kp - 0.05;
    std::cout << "current depth kp value: " + std::to_string(depth.kp)
              << std::endl;
  } else if (key == 'e') {
    depth.ki = depth.ki + 0.005;
    std::cout << "current depth ki value: " + std::to_string(depth.ki)
              << std::endl;
  } else if (key == 'd') {
    depth.ki = depth.ki - 0.001;
    std::cout << "current depth ki value: " + std::to_string(depth.ki)
              << std::endl;
  } else if (key == 'r') {
    depth.kd = depth.kd + 0.1;
    std::cout << "current depth kd value: " + std::to_string(depth.kd)
              << std::endl;
  } else if (key == 'f') {
    depth.kd = depth.kd - 0.1;
    std::cout << "current depth kd value: " + std::to_string(depth.kd)
              << std::endl;
  }
}

void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
  bool armed = msg->arm; // useless
  double depth_external = msg->external_pressure;
  int yaw_heading = msg->heading;

  depth_error = depth_setpoint - depth_external;
  yaw_error = yaw_setpoint - yaw_heading;
}

int main(int argc, char **argv) {
  // ROS2 Node Initialization
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("depth_tuner_controller");

  // ROS2 Publisher
  auto pwm_publisher = node->create_publisher<custom_msgs::msg::Commands>(
      "/master/commands", 10);

  // ROS2 Subscribers
  auto keys_subscriber =
      node->create_subscription<std_msgs::msg::Char>("keys", 10, keys_callback);

  auto telemetry_sub = node->create_subscription<custom_msgs::msg::Telemetry>(
      "/master/telemetry", 10, telemetryCallback);

  // Control Parameters Definition

  // Depth
  depth.kp = 5.8;
  depth.ki = 0;
  depth.kd = 20;
  depth.base_offset = 1580;
  depth_setpoint = 1069;

  yaw.kp = 0;
  yaw.ki = 0;
  yaw.kd = 0;
  yaw.base_offset = 1500; // this is 1500 only
  yaw_setpoint = 90;      // see headings dumdums

  // Arm Disarm Parameter
  bool arm = false;
  rclcpp::Time init_time = rclcpp::Clock().now();
  cmd_pwm.arm = false;

  rclcpp::Rate rate(10); // 10 Hz loop rate

  while (rclcpp::ok()) {
    if (software_arm_flag == true) {
      cmd_pwm.mode = "STABILIZE";
      rclcpp::Time time_now = rclcpp::Clock().now();
      cmd_pwm.arm = true;
      if (software_arm_flag == true) {
        float pid_depth = depth.pid_control(
            depth_error, (time_now - init_time).seconds(), false);

        float pid_yaw =
            yaw.pid_control(yaw_error, (time_now - init_time).seconds(), false);
        std::cout << " " << pid_depth << " " << pid_yaw << "\n";
        cmd_pwm.forward = 1500;
        cmd_pwm.lateral = 1500;
        cmd_pwm.thrust = pid_depth;
        cmd_pwm.yaw = 1500;
        std::cout << "sinking";
      }
    } else {
      depth.emptyError();
      cmd_pwm.arm = false;
      cmd_pwm.mode = "STABILIZE";
      cmd_pwm.forward = 1500;
      cmd_pwm.lateral = 1500;
      cmd_pwm.thrust = 1500;
      cmd_pwm.yaw = 1500;
    }
    pwm_publisher->publish(cmd_pwm);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
