#pragma once
#include <rclcpp/rclcpp.hpp>
#include <openvino/openvino.hpp>
#include <string>

void topic_camera_yolov11(std::shared_ptr<rclcpp::Node> nh, ov::Core &core,
						  const std::string &topic);
void cv_camera_yolov11(std::shared_ptr<rclcpp::Node> nh, ov::Core &core);