#pragma once
#include <libuvc/libuvc.h>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

struct CameraInfo {
    int vendor_id;
    int product_id;
    const char* serial_no;
    bool print_info; // Print device info
};

struct StreamInfo {
    int width;
    int height;
    uvc_frame_format frame_format;
    bool display_to_screen;
    bool create_server;
    int fps;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    std::shared_ptr<rclcpp::Node> node;
};

auto list_devices(uvc_context_t *ctx) -> int;
auto begin_streaming(uvc_context_t *ctx, CameraInfo csel, StreamInfo streaminfo) -> int;