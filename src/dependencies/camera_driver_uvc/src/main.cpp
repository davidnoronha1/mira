#include <libuvc/libuvc.h>
#include <cstdlib>
#include "main.h"
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/qos.hpp>
#include "camera_driver_uvc/clog.hpp"

auto main(int argc, char *argv[]) -> int {
    try {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("camera_driver_uvc");

    uvc_context_t *ctx = nullptr;
    uvc_error_t res;

    // Initialize context (libusb context handled internally)
    res = uvc_init(&ctx, NULL);
    if (res != UVC_SUCCESS) {
        plog::error() << "uvc_init failed: " << uvc_strerror(res);
        return EXIT_FAILURE;
    }

    bool stream = nh->declare_parameter("stream", false);
    std::string camera_info_url = nh->declare_parameter("camera_info_url", "");
    std::string camera_label = nh->declare_parameter("label", "camera");
    camera_info_manager::CameraInfoManager cinfo(nh.get(), camera_label, camera_info_url);

    if (!camera_info_url.empty()) {
        try {
            cinfo.getCameraInfo();
        } catch (const std::exception &e) {
            plog::warn() << "Failed to load camera info from URL: " << camera_info_url << " (" << e.what() << ")";
        }
    }

    list_devices(ctx);

    if (stream) {
        int vendor_id = nh->declare_parameter("vendor_id", -1);
        int product_id = nh->declare_parameter("product_id", -1);

        bool display_to_screen = nh->declare_parameter("display", false);
        int width = nh->declare_parameter("image_width", 640);
        int height = nh->declare_parameter("image_height", 480);

        std::string wanted_frame_format = nh->declare_parameter("frame_format", std::string("MJPEG"));
        enum uvc_frame_format frame_format = UVC_FRAME_FORMAT_MJPEG;
        if (wanted_frame_format == "YUYV") {
            frame_format = UVC_FRAME_FORMAT_YUYV;
        } else if (wanted_frame_format == "H264") {
            frame_format = UVC_FRAME_FORMAT_H264;
        } else if (wanted_frame_format == "MJPEG") {
            frame_format = UVC_FRAME_FORMAT_MJPEG;
        }

        int fps = nh->declare_parameter("framerate", 30);

        std::string serial_no = nh->declare_parameter("serial_no", std::string());

        if (vendor_id == -1 || product_id == -1) {
            plog::error() << "Vendor ID and Product ID must be specified. (" << vendor_id << ", " << product_id << ")";
            uvc_exit(ctx);
            return EXIT_FAILURE;
        }

        begin_streaming(ctx,
            CameraInfo { vendor_id, product_id, serial_no.empty() ? nullptr : serial_no.c_str(), true },
            StreamInfo {
                width,
                height,
                frame_format,
                display_to_screen,
                nh->declare_parameter("create_server", false) && frame_format == UVC_FRAME_FORMAT_MJPEG,
                fps,
                nh->create_publisher<sensor_msgs::msg::Image>(camera_label + "/image_raw", rclcpp::SensorDataQoS()),
                nh
            });
    }

    uvc_exit(ctx);
    } catch (const std::exception &e) {
        plog::exception() << "Exception: " << e.what();
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
