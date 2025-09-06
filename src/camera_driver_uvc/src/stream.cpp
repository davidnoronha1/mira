#include "libuvc/libuvc.h"
#include "main.h"
#include "util.h"
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unistd.h>

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
static void cb(uvc_frame_t *frame, void *ptr) {
  StreamInfo sinfo = *((StreamInfo *)ptr);
  // We have only implemented support for mjpeg
  if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Decode MJPEG frame to OpenCV Mat and display
    if (sinfo.display_to_screen || sinfo.create_server) {
      std::vector<uchar> jpeg_data((uchar *)frame->data,
                                   (uchar *)frame->data + frame->data_bytes);
      cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

      auto msg = sensor_msgs::msg::Image();
      msg.header.stamp = rclcpp::Clock().now();
      msg.height = img.rows;
      msg.width = img.cols;
      msg.encoding = "bgr8"; // OpenCV uses BGR format
      msg.is_bigendian = 0U;
      msg.step = img.cols * img.elemSize();
      msg.data.resize(img.total() * img.elemSize());
      std::memcpy(msg.data.data(), img.data, msg.data.size());
      sinfo.publisher->publish(msg);

      if (sinfo.display_to_screen && !img.empty()) {
        cv::imshow("MJPEG Stream", img);
        cv::waitKey(1); // Needed to update the window
      }
    }
  }
}

int begin_streaming(uvc_context_t *ctx, CameraInfo csel,
                    StreamInfo streaminfo, rclcpp::Node::SharedPtr node) {

  RCLCPP_INFO(node->get_logger(), "Starting streaming with the following parameters:");
  RCLCPP_INFO(node->get_logger(), "Camera Info:");
  RCLCPP_INFO(node->get_logger(), "  Vendor ID: 0x%04x", csel.vendor_id);
  RCLCPP_INFO(node->get_logger(), "  Product ID: 0x%04x", csel.product_id);
  if (csel.serial_no) {
    RCLCPP_INFO(node->get_logger(), "  Serial No: %s", csel.serial_no);
  } else {
    RCLCPP_INFO(node->get_logger(), "  Serial No: (none)");
}

  RCLCPP_INFO(node->get_logger(), "Stream Info:");
  RCLCPP_INFO(node->get_logger(), "  Width: %d", streaminfo.width);
  RCLCPP_INFO(node->get_logger(), "  Height: %d", streaminfo.height);
  RCLCPP_INFO(node->get_logger(), "  FPS: %d", streaminfo.fps);
  RCLCPP_INFO(node->get_logger(), "  Display to Screen: %s",
         streaminfo.display_to_screen ? "Yes" : "No");
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev, csel.vendor_id, csel.product_id,
      csel.serial_no); /* filter devices: vendor_id, product_id, "serial_num" */

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
    RCLCPP_ERROR(node->get_logger(), 
                 "No device found with Vendor ID: 0x%04x, Product ID: 0x%04x\nUse the --list-devices option to see available devices.",
                 csel.vendor_id, csel.product_id);
  } else {
    RCLCPP_INFO(node->get_logger(), "Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
      if (res == UVC_ERROR_ACCESS) {
        RCLCPP_ERROR(
            node->get_logger(),
            "failed to access device:\n"
            "===========================================================\n"
            "Access denied to device, try running as root or with sudo.\n"
            "Or for temporary access run the following command:\n"
            "sudo chmod o+w /dev/bus/usb/%03d/%03d\n"
            "============================================================\n",
            uvc_get_bus_number(dev), uvc_get_device_address(dev));
      }
    } else {
      RCLCPP_INFO(node->get_logger(), "Device opened");

      /* Print out a message containing all the information that libuvc
       * knows about the device */
      if (csel.print_info)
        uvc_print_diag(devh, stderr);

      const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      enum uvc_frame_format frame_format;
      int width = streaminfo.width;
      int height = streaminfo.height;
      int fps = streaminfo.fps;

      frame_format = UVC_FRAME_FORMAT_MJPEG;

      if (frame_desc) {
        width = frame_desc->wWidth;
        height = frame_desc->wHeight;
        fps = 10000000 / frame_desc->dwDefaultFrameInterval;
      }

      RCLCPP_INFO(node->get_logger(),
          "First format: (%4s) %dx%d %dfps",
          format_desc->fourccFormat, width, height, fps);

      /* Try to negotiate first stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl,                     /* result stored in ctrl */
          frame_format, width, height, fps /* width, height, fps */
      );

      /* Print out the result */
      if (csel.print_info)
        uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode");
        RCLCPP_ERROR(node->get_logger(),
               "Device doesn't provide a matching stream\n"
               "Consult v4l2-ctl --all to find compatible formats for this device");
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void *) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, &streaminfo, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          RCLCPP_INFO(node->get_logger(), "Started Streaming...");

          /* enable auto exposure - see uvc_set_ae_mode documentation */
          RCLCPP_INFO(node->get_logger(), "Enabling auto exposure ...");
          const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          if (res == UVC_SUCCESS) {
            RCLCPP_INFO(node->get_logger(), " ... enabled auto exposure");
          } else if (res == UVC_ERROR_PIPE) {
            /* this error indicates that the camera does not support the full AE
             * mode; try again, using aperture priority mode (fixed aperture,
             * variable exposure time) */
            RCLCPP_INFO(node->get_logger(), " ... full AE not supported, trying aperture priority mode");
            const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
            res =
                uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
            if (res < 0) {
              uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture "
                              "priority mode");
            } else {
              RCLCPP_INFO(node->get_logger(), " ... enabled aperture priority auto exposure mode");
            }
          } else {
            uvc_perror(
                res,
                " ... uvc_set_ae_mode failed to enable auto exposure mode");
          }


          RCLCPP_INFO(node->get_logger(), "Press CTRL-C to stop streaming.");
          pause();

          uvc_stop_streaming(devh);
          RCLCPP_INFO(node->get_logger(), "Done streaming.");
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      RCLCPP_INFO(node->get_logger(), "Device closed");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  return 0;
}