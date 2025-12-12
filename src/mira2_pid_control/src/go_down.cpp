#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <std_msgs/msg/char.hpp>

bool armed_flag = false;
rclcpp::Time start_time;

void key_callback(const std_msgs::msg::Char::SharedPtr msg) {
  char key = msg->data;

  if (key == 'p') {
    armed_flag = true;
    start_time = rclcpp::Clock().now();
    std::cout << "ARMED + STARTING 2s DOWNWARD THRUST\n";
  }

  if (key == 'q') {
    armed_flag = false;
    std::cout << "DISARMED\n";
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("simple_depth_bump_keys");
  auto pub = node->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);

  auto key_sub = node->create_subscription<std_msgs::msg::Char>(
      "keys", 10, key_callback);

  custom_msgs::msg::Commands cmd;
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    if (!armed_flag) {
      // Disarmed state
      cmd.arm = false;
      cmd.mode = "ALT_HOLD";
      cmd.forward = 1500;
      cmd.lateral = 1500;
      cmd.yaw = 1500;
      cmd.thrust = 1500;
    } 
    else {
      // Armed
      cmd.arm = true;
      cmd.mode = "ALT_HOLD";
      cmd.forward = 1500;
      cmd.lateral = 1500;
      cmd.yaw = 1500;

      double elapsed = (rclcpp::Clock().now() - start_time).seconds();

      if (elapsed < 2.0) {
        cmd.thrust = 1400;        // give downward push
        std::cout << "descending...\n";
      } else {
        cmd.thrust = 1500;        // neutral, depth-hold should stabilize
        std::cout << "holding depth\n";
      }
    }

    pub->publish(cmd);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
