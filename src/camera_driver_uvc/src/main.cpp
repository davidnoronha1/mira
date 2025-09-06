#include <stdio.h>
#include <libuvc/libuvc.h>
#include <cstdlib>
#include "main.h"

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("camera_driver_uvc");

    uvc_context_t *ctx;
    uvc_error_t res;

    // Initialize context (libusb context handled internally)
    res = uvc_init(&ctx, NULL);
    if (res != UVC_SUCCESS) {
        uvc_perror(res, "uvc_init");
        return EXIT_FAILURE;
    }

    // bool list_devices_opt = nh->declare_parameter("list_devices", false);
    bool stream = nh->declare_parameter("stream", false);

    // if (list_devices_opt) {
        list_devices(ctx);
    // } 
    
    if (stream) {
        int vendor_id = nh->declare_parameter("vendor_id", -1);
        int product_id = nh->declare_parameter("product_id", -1);

        bool display_to_screen = nh->declare_parameter("display", false);
        int width = nh->declare_parameter("image_width", 1920);
        int height = nh->declare_parameter("image_height", 1080);

        std::string wanted_frame_format = nh->declare_parameter("frame_format", std::string("MJPEG"));
        enum uvc_frame_format frame_format;
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
            fprintf(stderr, "Vendor ID and Product ID must be specified. (%d, %d)\n", vendor_id, product_id);
            uvc_exit(ctx);
            return EXIT_FAILURE;
        }

        begin_streaming(ctx,
            CameraInfo { vendor_id, product_id, serial_no.empty() ? nullptr : serial_no.c_str(), nh->declare_parameter("verbose", false) },
            StreamInfo {
                width,
                height,
                frame_format,
                display_to_screen,
                nh->declare_parameter("create_server", false) && frame_format == UVC_FRAME_FORMAT_MJPEG,
                fps,
                nh->create_publisher<sensor_msgs::msg::Image>("camera/image", 0)
            });
    }

    uvc_exit(ctx);

    return EXIT_SUCCESS;
}