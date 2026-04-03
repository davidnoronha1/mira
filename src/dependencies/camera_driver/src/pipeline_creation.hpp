#ifndef CAMERA_DRIVER_PIPELINE_CREATION_HPP
#define CAMERA_DRIVER_PIPELINE_CREATION_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>

enum class EncoderType { X264, QSV, NVENC, V4L2 };

struct EncoderInfo {
  EncoderType type;
  std::string name;        // GStreamer element name
  std::string extra_props; // properties appended after bitrate
};

std::string caps_for_encoder(EncoderType t);

bool gst_element_exists(const std::string &name);

EncoderInfo detect_best_encoder(long bitrate, long framerate, rclcpp::Logger logger);

std::string build_pipeline(const std::string &fmt,
                           const std::string &device_path,
                           long width, long height,
                           long framerate, long bitrate,
                           rclcpp::Logger logger);

std::string gst_caps_for_format(const std::string &fmt);

#endif // CAMERA_DRIVER_PIPELINE_CREATION_HPP
