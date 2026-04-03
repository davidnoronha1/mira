#include "pipeline_creation.hpp"
#include <gst/gst.h>
#include <sstream>
#include <strings.h> // for strcasecmp

std::string caps_for_encoder(EncoderType t) {
  switch (t) {
  case EncoderType::X264:
    return "video/x-raw,format=I420";
  case EncoderType::QSV:
    return "video/x-raw,format=NV12";
  case EncoderType::NVENC:
    return "video/x-raw,format=NV12";
  case EncoderType::V4L2:
    return "video/x-raw,format=NV12";
  }
  return "video/x-raw,format=I420";
}

bool gst_element_exists(const std::string &name) {
  GstElementFactory *f = gst_element_factory_find(name.c_str());
  if (!f)
    return false;
  gst_object_unref(f);
  return true;
}

EncoderInfo detect_best_encoder(long bitrate, long framerate, rclcpp::Logger logger) {
  // ── Intel QSV ──────────────────────────────────────────────────────────
  if (gst_element_exists("qsvh264enc")) {
    RCLCPP_INFO(logger, "Encoder: Intel QSV (qsvh264enc)");
    return {EncoderType::QSV, "qsvh264enc",
            " bitrate=" + std::to_string(bitrate) +
                " target-usage=7"
                " gop-size=" +
                std::to_string(framerate) +
                " b-frames=0"
                " low-latency=true"
                " rate-control=cbr"};
  }

  // ── NVIDIA NVENC ───────────────────────────────────────────────────────
  if (gst_element_exists("nvh264enc")) {
    RCLCPP_INFO(logger, "Encoder: NVIDIA NVENC (nvh264enc)");
    return {EncoderType::NVENC, "nvh264enc",
            " bitrate=" + std::to_string(bitrate) +
                " preset=low-latency-hp"
                " gop-size=" +
                std::to_string(framerate) +
                " bframes=0"
                " rc-mode=cbr"
                " zerolatency=true"};
  }

  // ── V4L2 M2M hardware (Raspberry Pi / i.MX / Rockchip …) ──────────────
  if (gst_element_exists("v4l2h264enc")) {
    RCLCPP_INFO(logger, "Encoder: V4L2 M2M hardware (v4l2h264enc)");
    return {EncoderType::V4L2, "v4l2h264enc",
            " extra-controls=\"encode,video_bitrate=" +
                std::to_string(bitrate * 1000) + "\""};
  }

  // ── Software x264 fallback ─────────────────────────────────────────────
  RCLCPP_WARN(logger,
              "Encoder: software x264 (no hardware encoder found)");
  return {EncoderType::X264, "x264enc",
          " bitrate=" + std::to_string(bitrate) +
              " speed-preset=ultrafast"
              " tune=zerolatency"
              " key-int-max=" +
              std::to_string(framerate) +
              " bframes=0"
              " sliced-threads=true"
              " rc-lookahead=0"
              " sync-lookahead=0"
              " vbv-buf-capacity=0"};
}

std::string gst_caps_for_format(const std::string &fmt) {
  if (strcasecmp(fmt.c_str(), "MJPEG") == 0)
    return "image/jpeg";
  if (strcasecmp(fmt.c_str(), "YUYV") == 0)
    return "video/x-raw,format=YUY2";
  if (strcasecmp(fmt.c_str(), "H264") == 0)
    return "video/x-h264";
  if (strcasecmp(fmt.c_str(), "RGB") == 0)
    return "video/x-raw,format=RGB";
  if (strcasecmp(fmt.c_str(), "BGR") == 0)
    return "video/x-raw,format=BGR";
  if (strcasecmp(fmt.c_str(), "NV12") == 0)
    return "video/x-raw,format=NV12";
  return "image/jpeg"; // default fallback
}

std::string build_pipeline(const std::string &fmt,
                           const std::string &device_path,
                           long width, long height,
                           long framerate, long bitrate,
                           rclcpp::Logger logger) {
  std::string gst_fmt = gst_caps_for_format(fmt);
  std::ostringstream ss;

  if (strcasecmp(fmt.c_str(), "H264") == 0) {
    // ── H.264 passthrough: camera already encodes, skip re-encode ─────────
    // RTSP branch gets the raw H.264 bitstream directly.
    // ROS2 branch decodes to RGB for image messages.
    RCLCPP_INFO(logger,
                "Pipeline mode: H.264 passthrough (no re-encode)");
    ss << "( "
       << "v4l2src device=" << device_path << " ! "
       << gst_fmt << ",width=" << width << ",height=" << height
       << ",framerate=" << framerate << "/1 ! "
       << "h264parse ! "
       << "tee name=t "

       // Branch 1 – RTSP: pay the H.264 directly
       << "t. ! queue leaky=downstream max-size-buffers=2 ! "
       << "rtph264pay name=pay0 pt=96 config-interval=1 "

       // Branch 2 – ROS2: decode → RGB appsink
       << "t. ! queue leaky=downstream max-size-buffers=2 ! "
       << "avdec_h264 ! "
       << "videoconvert ! video/x-raw,format=RGB ! "
       << "appsink name=ros_sink max-buffers=2 drop=true sync=false "
       << ")";
  } else {
    // ── MJPEG / YUYV / raw: decode then encode to H.264 for RTSP ─────────
    auto best_enc = detect_best_encoder(bitrate, framerate, logger);

    if (strcasecmp(fmt.c_str(), "MJPEG") == 0) {
      RCLCPP_INFO(logger, "Pipeline mode: MJPEG → decode → %s", best_enc.name.c_str());
    } else {
      RCLCPP_INFO(logger, "Pipeline mode: raw → videoconvert → %s", best_enc.name.c_str());
    }

    ss << "( "
       << "v4l2src device=" << device_path << " ! "
       << gst_fmt << ",width=" << width << ",height=" << height
       << ",framerate=" << framerate << "/1 ! ";

    // Decode compressed input formats
    if (strcasecmp(fmt.c_str(), "MJPEG") == 0)
      ss << "jpegdec ! ";

    ss << "videoconvert ! "
       << "tee name=t "

       // Branch 1 – RTSP/H.264
       << "t. ! queue leaky=downstream max-size-buffers=2 ! "
       << "videoconvert ! " << caps_for_encoder(best_enc.type) << " ! "
       << best_enc.name << " " << best_enc.extra_props << " ! "
       << "rtph264pay name=pay0 pt=96 config-interval=1 "

       // Branch 2 – ROS2 appsink (RGB, no extra encoding)
       << "t. ! queue leaky=downstream max-size-buffers=2 ! "
       << "videoconvert ! video/x-raw,format=RGB ! "
       << "appsink name=ros_sink max-buffers=2 drop=true sync=false "
       << ")";
  }

  return ss.str();
}
