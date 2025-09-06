#pragma once
#include <libuvc/libuvc.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

typedef struct {
    int vendor_id;
    int product_id;
    const char* serial_no;
    bool print_info; // Print device info
} CameraInfo;

typedef struct {
    int width;
    int height;
    int frame_format;
    bool display_to_screen;
    bool create_server;
    int fps;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
} StreamInfo;

int list_devices(uvc_context_t *ctx);
int begin_streaming(uvc_context_t *ctx, CameraInfo csel, StreamInfo streaminfo);