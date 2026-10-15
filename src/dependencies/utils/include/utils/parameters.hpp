#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace utils {

// Schemas stay in their components. ROS resolves files and CLI overrides;
// existing declarations are read without resetting their value or descriptor.
template <typename T>
T parameter(rclcpp::Node &node, const std::string &name, const T &fallback,
            bool read_only = false) {
  if (!node.has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = read_only;
    return node.declare_parameter<T>(name, fallback, descriptor);
  }
  return node.get_parameter(name).get_value<T>();
}

template <typename T, typename Validator>
T checked_parameter(rclcpp::Node &node, const std::string &name, const T &fallback,
                    Validator validate, bool read_only = false) {
  const auto value = parameter(node, name, fallback, read_only);
  if (!validate(value)) {
    throw std::invalid_argument(name + ": invalid parameter value");
  }
  return value;
}

inline bool positive(double value) { return std::isfinite(value) && value > 0; }

}  // namespace utils
