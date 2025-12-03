#pragma once

#include <memory>
#include <openvino/openvino.hpp>
#include <rclcpp/rclcpp.hpp>

void list_devices(std::shared_ptr<rclcpp::Node> nh, ov::Core& core);
void cv_camera_depth(std::shared_ptr<rclcpp::Node> nh, ov::Core& core);
void topic_camera_depth(std::shared_ptr<rclcpp::Node> nh, ov::Core& core, const std::string &topic);