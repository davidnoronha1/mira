#include "app.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	ov::Core core;
	auto nh = rclcpp::Node::make_shared("vision_depth_node");

	auto verbose = nh->declare_parameter("verbose", false);

	if (verbose) {
		list_devices(nh, core);
	}

	auto use_cv_camera = nh->declare_parameter("webcam", false);

	if (use_cv_camera) {
		cv_camera_depth(nh, core);
	}

	auto topic = nh->declare_parameter("image_topic", "");

	if (!topic.empty()) {
		topic_camera_depth(nh, core, topic);
	}

	rclcpp::shutdown();
}