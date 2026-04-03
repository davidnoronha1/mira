#ifndef CAMERA_DRIVER_DEVICE_FINDING_HPP
#define CAMERA_DRIVER_DEVICE_FINDING_HPP

#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>

// Helper to match USB port strings
bool usb_port_matches(const gchar *prop_value, const std::string &usb_port);

// Find a camera whose physical USB port matches `usb_port`.
GstDevice *find_camera_by_usb_port(const std::string &usb_port,
                                   std::string &device_path,
                                   rclcpp::Logger logger);

GstDevice *find_camera_by_id(long vendor, long product,
                             const std::string &serial,
                             rclcpp::Logger logger);

GstDevice *find_camera_by_path(std::string &device_path,
                               rclcpp::Logger logger);

GstDevice *find_camera_any(std::string &device_path,
                           rclcpp::Logger logger);

void query_camera_capabilities(GstDevice *dev, std::set<std::string> &formats,
                               std::set<std::pair<int, int>> &resolutions,
                               rclcpp::Logger logger);

#endif // CAMERA_DRIVER_DEVICE_FINDING_HPP
