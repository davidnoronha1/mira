#include <chrono>
#include <functional>
#include <memory>
#include <new>
#include <string>

#include "app.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "vision_boundingbox/clog.hpp"

using namespace std::chrono_literals;

auto main(int argc, char **argv) -> int {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vision_boundingbox_node");
  rclcpp::Rate loop_rate(10ms);

  auto webcam = node->declare_parameter("webcam", false);
  auto topic = node->declare_parameter("image_topic", std::string(""));

  auto core = ov::Core();

  if (webcam) {
    cv_camera_yolov11(node, core);
  } else if (!topic.empty()) {
    topic_camera_yolov11(node, core, topic);
  } else {
    RCLCPP_ERROR(node->get_logger(), "No input source specified. Use --ros-args -p webcam:=true or -p image_topic:=<topic_name>");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}