#include "app.hpp"
#include <cstdlib>
#include <exception>
#include <rclcpp/utilities.hpp>
#include "vision_depth/clog.hpp"

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node_h = rclcpp::Node::make_shared("vision_depth_node");
    ov::Core core;

    auto verbose = node_h->declare_parameter("verbose", false);

    if (verbose) {
      list_devices(node_h, core);
    }

    auto use_cv_camera = node_h->declare_parameter("webcam", false);
    auto topic = node_h->declare_parameter("image_topic", "");

    if (use_cv_camera) {
	  plog::info() << "Starting in webcam mode";
      cv_camera_depth(node_h, core);
    }

    if (!use_cv_camera && !topic.empty()) {
	  plog::info() << "Starting in ROS2 topic mode, subscribing to " << topic;
      topic_camera_depth(node_h, core, topic);
    }

  } catch (std::exception &e) {
	plog::exception() << "Fatal error: " << e.what();
	return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}