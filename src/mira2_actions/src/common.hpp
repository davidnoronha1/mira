#pragma once
#include <custom_msgs/msg/commands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/telemetry.hpp>

struct ROSState {
	public:
	custom_msgs::msg::Telemetry telemetry;
	rclcpp::Node::SharedPtr node;
	rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr cmd_publisher;

	private:
	rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_sub;

	public:
	ROSState (rclcpp::Node::SharedPtr node) : node(node) {
		telemetry_sub = node->create_subscription<custom_msgs::msg::Telemetry>(
			"/master/telemetry", 1,
			[this](const custom_msgs::msg::Telemetry::SharedPtr msg) {
				telemetry = *msg;
			});
		cmd_publisher = node->create_publisher<custom_msgs::msg::Commands>("/master/commands", 1);
	}
};