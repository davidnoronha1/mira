#include "app.hpp"
#include "vision_depth/clog.hpp"

void list_devices(std::shared_ptr<rclcpp::Node> nh, ov::Core &core) {
  try {
    auto devices = core.get_available_devices();
    plog::info() << "Available devices:";
    for (const auto &device : devices) {
      std::cout << " - " << device << '\n';
      auto properties = core.get_property(device, ov::supported_properties);
      for (const auto &prop : properties) {
      if (prop == ov::supported_properties) {
        continue;
      }
      std::cout << "    - " << prop << ": "
             << core.get_property(device, prop).as<std::string>() << '\n';
      }
    }
  } catch (const std::exception &e) {
    plog::exception() << "Error listing devices: " << e.what();
  }
}