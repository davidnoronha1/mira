#include "app.hpp"

void list_devices(std::shared_ptr<rclcpp::Node> nh, ov::Core &core) {
  try {
    auto devices = core.get_available_devices();
    RCLCPP_INFO(nh->get_logger(), "Available devices:");
    for (const auto &device : devices) {
      RCLCPP_INFO(nh->get_logger(), " - %s", device.c_str());
      auto properties = core.get_property(device, ov::supported_properties);
      for (const auto &prop : properties) {
        if (prop == ov::supported_properties) {
          continue;
        }
        RCLCPP_INFO(nh->get_logger(), "    - %s: %s", prop.c_str(),
                    core.get_property(device, prop).as<std::string>().c_str());
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(nh->get_logger(), "Error listing devices: %s", e.what());
  }
}