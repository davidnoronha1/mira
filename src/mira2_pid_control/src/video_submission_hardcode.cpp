#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <control_utils/control_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// Control parameters and PWM Commands
bool software_arm_flag = false;
custom_msgs::msg::Commands cmd_pwm;
PID_Controller depth;
double depth_error;
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
  }
}

void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
  double depth_external = msg->external_pressure;
  depth_error = 1075 - depth_external;
}

int main(int argc, char **argv) {
  // ROS 2 Node Declaration
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("depth_tuner_controller");

  // ROS 2 Publisher
  auto pwm_publisher = node->create_publisher<custom_msgs::msg::Commands>(
      "/master/commands", 10);

  // ROS 2 Subscriber
  auto keys_subscriber =
      node->create_subscription<std_msgs::msg::Char>("keys", 10, keys_callback);

  auto telemetry_sub = node->create_subscription<custom_msgs::msg::Telemetry>(
      "/master/telemetry", 10, telemetryCallback);

  // Depth
  depth.kp = -2;     // -2;
  depth.ki = -0.2;   // -0.2;
  depth.kd = -10.69; // -15.69;

  RCLCPP_INFO(node->get_logger(), "lilbitchlaky");

  // Arm Disarm Parameter
  bool arm = false;
  rclcpp::Time init_time = rclcpp::Clock().now();
  cmd_pwm.arm = false;

  rclcpp::Rate rate(10); // 10 Hz loop rate
  while (rclcpp::ok()) {
    if (software_arm_flag == true) {
      cmd_pwm.mode = "STABILIZE";
      rclcpp::Time time_now = rclcpp::Clock().now();
      cmd_pwm.arm = false;
      if (software_arm_flag == true) {
        float pid_depth = depth.pid_control(
            depth_error, (time_now - init_time).seconds(), false);
        std::cout << ((time_now - start_routine).seconds()) << "\n";
        float delay = 69.0;
        if ((time_now - start_routine).seconds() < delay) {
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          depth.emptyError();
        } else if ((time_now - start_routine).seconds() < (delay + 5)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "sinking ";
        } else if ((time_now - start_routine).seconds() < (delay + 10.50)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1800;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "stabilize surge ";
        } else if ((time_now - start_routine).seconds() < (delay + 13.10)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1800;
          std::cout << "stabilize yaw ";
        } else if ((time_now - start_routine).seconds() < (delay + 16.30)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1800;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "stabilize surge ";
        } else if ((time_now - start_routine).seconds() < (delay + 16.35)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "disarm";
        } else if ((time_now - start_routine).seconds() < (delay + 16.40)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "change to manual";
        } else if ((time_now - start_routine).seconds() < (delay + 19.50)) {
          cmd_pwm.arm = true;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1800;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          cmd_pwm.roll = 1850;
          std::cout << " surge with roll  and manual";
        } else if ((time_now - start_routine).seconds() < (delay + 19.55)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "disarm";
        } else if ((time_now - start_routine).seconds() < (delay + 19.60)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "change to stabilize";
        } else if ((time_now - start_routine).seconds() < (delay + 21.60)) {
          cmd_pwm.arm = true;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          cmd_pwm.roll = 1500;
          std::cout << "forward";
        } else if ((time_now - start_routine).seconds() < (delay + 25)) {
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "wait";
        } else {
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "pusi";
        }
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
