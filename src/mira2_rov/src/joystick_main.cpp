#include <custom_msgs/msg/commands.h>
#include <mira2_rov/joystick_utils.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
