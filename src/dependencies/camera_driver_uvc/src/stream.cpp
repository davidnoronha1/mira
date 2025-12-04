#include "camera_driver_uvc/clog.hpp"
#include "libuvc/libuvc.h"
#include "main.h"
#include "util.h"
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unistd.h>
#include <utility>

static void cb(uvc_frame_t *frame, void *ptr) {
  using namespace std::chrono;
  static auto last_time = steady_clock::now();
  static int frame_count = 0;
  static double fps = 0.0;

  StreamInfo sinfo = *((StreamInfo *)ptr);
  if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    if (sinfo.display_to_screen || sinfo.create_server) {
      cv::Mat jpeg_data(1, frame->data_bytes, CV_8UC1, frame->data);
      cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

      auto msg = sensor_msgs::msg::Image();
      msg.header.stamp = rclcpp::Clock().now();
      msg.height = img.rows;
      msg.width = img.cols;
      msg.encoding = "bgr8";
      msg.is_bigendian = 0U;
      msg.step = img.cols * img.elemSize();
      msg.data.resize(img.total() * img.elemSize());
      std::memcpy(msg.data.data(), img.data, msg.data.size());
      sinfo.publisher->publish(msg);

      if (sinfo.display_to_screen && !img.empty()) {
        frame_count++;
        auto now = steady_clock::now();
        auto elapsed = duration_cast<seconds>(now - last_time).count();
        if (elapsed >= 1) {
          fps = frame_count / (double)elapsed;
          frame_count = 0;
          last_time = now;
        }

        if (fps > 0.0) {
          cv::putText(img, "FPS: " + std::to_string((int)fps),
                      cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                      cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("MJPEG Stream", img);
        cv::waitKey(1);
      }
    }
  }
}

auto begin_streaming(uvc_context_t *ctx, CameraInfo csel, StreamInfo streaminfo)
    -> int {

  auto node = streaminfo.node;

  plog::info() << "Starting streaming with the following parameters:";
  plog::info() << "Camera Info:";
  plog::info() << "- Vendor ID: 0x" << std::hex << csel.vendor_id;
  plog::info() << "- Product ID: 0x" << std::hex << csel.product_id;
  if (csel.serial_no) {
    plog::info() << "- Serial No: " << csel.serial_no;
  } else {
    plog::info() << "- Serial No: (none)";
  }

  plog::info() << "Stream Info:";
  plog::info() << "- Width: " << streaminfo.width;
  plog::info() << "- Height: " << streaminfo.height;
  plog::info() << "- FPS: " << streaminfo.fps;
  plog::info() << "- Display to Screen: "
               << (streaminfo.display_to_screen ? "Yes" : "No");

  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  res = uvc_find_device(ctx, &dev, csel.vendor_id, csel.product_id,
                        csel.serial_no);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device");
    plog::exception()
        << "No device found with Vendor ID: 0x" << std::hex << csel.vendor_id
        << ", Product ID: 0x" << std::hex << csel.product_id
        << "\nUse the --list-devices option to see available devices.";
  } else {
    plog::info() << "Device found";

    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open");
      if (res == UVC_ERROR_ACCESS) {
        plog::exception()
            << "failed to access device:\n"
            << "===========================================================\n"
            << "Access denied to device, try running as root or with sudo.\n"
            << "Or for temporary access run the following command:\n"
            << "sudo chmod o+w /dev/bus/usb/" << std::setw(3)
            << std::setfill('0') << uvc_get_bus_number(dev) << "/"
            << std::setw(3) << std::setfill('0') << uvc_get_device_address(dev)
            << "\n============================================================"
               "\n";
      }
    } else {
      plog::info() << "Device opened";

      if (csel.print_info)
        uvc_print_diag(devh, stderr);

      const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      enum uvc_frame_format frame_format;
      int width = streaminfo.width;
      int height = streaminfo.height;
      int fps = streaminfo.fps;

      frame_format = streaminfo.frame_format;

      if (frame_desc) {
        width = frame_desc->wWidth;
        height = frame_desc->wHeight;
        fps = 10000000 / frame_desc->dwDefaultFrameInterval;
      }

      plog::info() << "First format: (" << format_desc->fourccFormat << ") "
                   << width << "x" << height << " " << fps << "fps";

      res = uvc_get_stream_ctrl_format_size(devh, &ctrl, frame_format, width,
                                            height, fps);

      if (csel.print_info)
        uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode");
        plog::exception() << "Device doesn't provide a matching stream\n"
                          << "Consult v4l2-ctl --all to find compatible "
                             "formats for this device";
      } else {
        res = uvc_start_streaming(devh, &ctrl, cb, &streaminfo, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming");
        } else {
          plog::info() << "Started Streaming...";

          plog::info() << "Enabling auto exposure ...";
          const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          if (res == UVC_SUCCESS) {
            plog::info() << " ... enabled auto exposure";
          } else if (res == UVC_ERROR_PIPE) {
            plog::warn()
                << " ... full AE not supported, trying aperture priority mode";
            const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
            res =
                uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
            if (res < 0) {
              uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture "
                              "priority mode");
            } else {
              plog::info()
                  << " ... enabled aperture priority auto exposure mode";
            }
          } else {
            uvc_perror(
                res,
                " ... uvc_set_ae_mode failed to enable auto exposure mode");
          }

          plog::info() << "Press CTRL-C to stop streaming.";
          rclcpp::spin(std::move(node));

          uvc_stop_streaming(devh);
          plog::info() << "Done streaming.";
        }
      }

      uvc_close(devh);
      plog::info() << "Device closed";
    }

    uvc_unref_device(dev);
  }

  return 0;
}
