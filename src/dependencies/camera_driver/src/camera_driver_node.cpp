// camera_rtsp_streamer_v6.cpp
//
// Single-pipeline architecture:
//
//  For MJPEG / YUYV cameras:
//   v4l2src → [jpegdec if MJPEG] → videoconvert → tee ┬→ queue → x264enc → rtph264pay (RTSP)
//                                                      └→ queue → videoconvert → RGB → appsink (ROS2)
//
//  For H.264 cameras (passthrough — no decode/re-encode):
//   v4l2src → h264parse → tee ┬→ queue → rtph264pay (RTSP)
//                             └→ queue → avdec_h264 → videoconvert → RGB → appsink (ROS2)
//
// The RTSP media factory owns and runs the pipeline. We hook the factory's
// "media-configure" signal to retrieve the appsink from inside that pipeline
// and attach our ROS2 callback — so the camera is opened exactly once and
// encoding happens exactly once regardless of the number of RTSP clients.

#include "device_finding.hpp"
#include "pipeline_creation.hpp"

#include <future>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/gstdevice.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <set>

const char *get_machine_ip() {
  auto x = getenv("MACHINE_IP");
  return x == nullptr ? "{MACHINE_IP}" : x;
}

class RTSPCameraStreamer : public rclcpp::Node {
public:
  RTSPCameraStreamer() : Node("rtsp_camera_streamer") {}

  // Called after make_shared() — safe to use shared_from_this() here.
  void initialize() {
    gst_init(nullptr, nullptr);

    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter("vendor_id", -1);
    declare_parameter("product_id", -1);
    declare_parameter("serial_no", "");
    declare_parameter("image_width", 640);
    declare_parameter("image_height", 480);
    declare_parameter("frame_format", "MJPEG");
    declare_parameter("framerate", 30);
    declare_parameter("port", 8554);
    declare_parameter("bitrate", 2000);
    declare_parameter("device_path", "");
    declare_parameter("ros_topic", "");
    declare_parameter("camera_frame_id", "camera");
    declare_parameter("usb_port", "");
    declare_parameter("camera_info_url", "");

    std::string usb_port = get_parameter("usb_port").as_string();
    std::string device_path = get_parameter("device_path").as_string();
    long vendor = get_parameter("vendor_id").as_int();
    long product = get_parameter("product_id").as_int();
    std::string serial = get_parameter("serial_no").as_string();
    long width = get_parameter("image_width").as_int();
    long height = get_parameter("image_height").as_int();
    std::string fmt = get_parameter("frame_format").as_string();
    long framerate = get_parameter("framerate").as_int();
    long port = get_parameter("port").as_int();
    long bitrate = get_parameter("bitrate").as_int();
    std::string camera_info_url = get_parameter("camera_info_url").as_string();

    frame_id_ = get_parameter("camera_frame_id").as_string();
    ros_topic_basename_ = get_parameter("ros_topic").as_string();
    if (ros_topic_basename_ == "") {
      ros_topic_basename_ = frame_id_;
    }

    constexpr const char *mount_point = "/image_rtsp";

    if (vendor != -1)
      RCLCPP_INFO(get_logger(), "Vendor ID:  0x%04lx", vendor);
    if (product != -1)
      RCLCPP_INFO(get_logger(), "Product ID: 0x%04lx", product);
    if (!serial.empty())
      RCLCPP_INFO(get_logger(), "Serial: %s", serial.c_str());

    // ── Device discovery ────────────────────────────────────────────────────
    GstDevice *target_device = nullptr;

    RCLCPP_INFO(get_logger(),
                "Use `gst-device-monitor-1.0 Video/Source` to list all "
                "compatible devices and their properties");

    RCLCPP_INFO(get_logger(), "Trying to find camera by USB port (%s)",
                usb_port.c_str());
    target_device = find_camera_by_usb_port(usb_port, device_path, get_logger());

    if (target_device == nullptr) {
      RCLCPP_INFO(
          get_logger(),
          "Trying to find camera by vendor (%lx), product (%lx), serial (%s)",
          vendor, product, serial.c_str());
      target_device = find_camera_by_id(vendor, product, serial, get_logger());
    }

    if (target_device == nullptr) {
      RCLCPP_INFO(get_logger(),
                  "Previous failed! Trying to find device by path (%s)",
                  device_path.c_str());
      target_device = find_camera_by_path(device_path, get_logger());
    }

    if (target_device == nullptr) {
      RCLCPP_INFO(get_logger(), "Previous failed! Trying to find any device!");
      target_device = find_camera_any(device_path, get_logger());
    }

    RCLCPP_INFO(get_logger(), "Using webcam: %s",
                target_device != nullptr
                    ? gst_device_get_display_name(target_device)
                    : device_path.c_str());

    // ── Capability negotiation ───────────────────────────────────────────────
    std::set<std::string> supported_formats;
    std::set<std::pair<int, int>> supported_resolutions;
    query_camera_capabilities(target_device, supported_formats,
                              supported_resolutions, get_logger());

    if (!supported_formats.empty()) {
      bool found = false;
      for (const auto &sf : supported_formats)
        if (strcasecmp(sf.c_str(), fmt.c_str()) == 0) {
          found = true;
          break;
        }
      if (!found) {
        // Prefer H264 passthrough if available, then MJPEG, then whatever
        if (supported_formats.count("H264"))
          fmt = "H264";
        else if (supported_formats.count("MJPEG"))
          fmt = "MJPEG";
        else
          fmt = *supported_formats.begin();
        RCLCPP_WARN(get_logger(), "Requested format not supported, using '%s'",
                    fmt.c_str());
      }
    }

    if (!supported_resolutions.empty()) {
      bool found = false;
      for (const auto &r : supported_resolutions)
        if (r.first == width && r.second == height) {
          found = true;
          break;
        }
      if (!found) {
        auto best = *supported_resolutions.begin();
        int target_area = width * height;
        int min_diff = std::abs(best.first * best.second - target_area);
        for (const auto &r : supported_resolutions) {
          int diff = std::abs(r.first * r.second - target_area);
          if (diff < min_diff) {
            min_diff = diff;
            best = r;
          }
        }
        RCLCPP_WARN(get_logger(),
                    "Resolution %ldx%ld not supported, using %dx%d", width,
                    height, best.first, best.second);
        width = best.first;
        height = best.second;
      }
    }

    if (target_device)
      gst_object_unref(target_device);
    auto *const machine_ip = get_machine_ip();

    // ── CameraInfoManager ────────────────────────────────────────────────────
    camera_info_manager_ =
        std::make_shared<camera_info_manager::CameraInfoManager>(
            this, frame_id_);

    if (!camera_info_url.empty()) {
      if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        RCLCPP_INFO(get_logger(), "Camera info loaded from: %s",
                    camera_info_url.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
                    "camera_info_url '%s' is invalid — publishing uncalibrated "
                    "CameraInfo",
                    camera_info_url.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(),
                  "No camera_info_url set — publishing uncalibrated CameraInfo");
    }

    // ── Log configuration ────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
                "============================================================");
    RCLCPP_INFO(get_logger(), "RTSP Stream Configuration:");
    RCLCPP_INFO(get_logger(),
                "============================================================");
    RCLCPP_INFO(get_logger(), "Device path:        %s", device_path.c_str());
    RCLCPP_INFO(get_logger(), "Image Width:        %ld px", width);
    RCLCPP_INFO(get_logger(), "Image Height:       %ld px", height);
    RCLCPP_INFO(get_logger(), "Frame Format:       %s", fmt.c_str());
    RCLCPP_INFO(get_logger(), "Framerate:          %ld fps", framerate);
    RCLCPP_INFO(get_logger(), "H.264 Bitrate:      %ld kbps", bitrate);
    RCLCPP_INFO(get_logger(), "RTSP Port:          %ld", port);
    RCLCPP_INFO(get_logger(), "RTSP Mount Point:   %s", mount_point);
    RCLCPP_INFO(get_logger(), "RTSP URL:           rtsp://%s:%ld%s", machine_ip,
                port, mount_point);
    RCLCPP_INFO(get_logger(), "ROS2 Image Topic:   %s", ros_topic_basename_.c_str());
    RCLCPP_INFO(get_logger(), "Camera Info URL:    %s",
                camera_info_url.empty() ? "(none)" : camera_info_url.c_str());
    RCLCPP_INFO(get_logger(),
                "Command to receive: ffplay -fflags nobuffer -flags low_delay "
                "-framedrop -vf 'setpts=0' rtsp://%s:%ld/%s",
                machine_ip, port, mount_point);
    RCLCPP_INFO(get_logger(),
                "============================================================");

    // ── image_transport publisher ────────────────────────────────────────────
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise(ros_topic_basename_ + "/image", 1);

    // ── camera_info publisher ─────────────────────────────────────────────────
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        ros_topic_basename_ + "/camera_info", rclcpp::SensorDataQoS());

    // ── Build the shared pipeline string ────────────────────────────────────
    std::string pipeline_str = build_pipeline(fmt, device_path, width, height,
                                              framerate, bitrate, get_logger());
    RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", pipeline_str.c_str());
    RCLCPP_INFO(get_logger(),
                "============================================================");

    // ── RTSP server ──────────────────────────────────────────────────────────
    server_ = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server_, std::to_string(port).c_str());

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server_);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, pipeline_str.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    g_signal_connect(factory, "media-configure",
                     G_CALLBACK(on_media_configure_static), this);

    gst_rtsp_mount_points_add_factory(mounts, mount_point, factory);
    g_object_unref(mounts);

    // ── GLib main loop ───────────────────────────────────────────────────────
    glib_context_ = g_main_context_new();
    loop_ = g_main_loop_new(glib_context_, FALSE);

    std::promise<void> loop_ready;
    auto loop_ready_fut = loop_ready.get_future();
    glib_thread_ = std::thread([this, p = std::move(loop_ready)]() mutable {
      GSource *idle = g_idle_source_new();
      g_source_set_callback(
          idle,
          [](gpointer data) -> gboolean {
            static_cast<std::promise<void> *>(data)->set_value();
            return G_SOURCE_REMOVE;
          },
          &p, nullptr);
      g_source_attach(idle, glib_context_);
      g_source_unref(idle);
      g_main_loop_run(loop_);
    });
    loop_ready_fut.wait();

    if (gst_rtsp_server_attach(server_, glib_context_) == 0)
      throw std::runtime_error("Failed to attach RTSP server");

    RCLCPP_INFO(
        get_logger(),
        "RTSP server started. Waiting for clients on rtsp://<host>:%ld%s", port,
        mount_point);
  }

  ~RTSPCameraStreamer() { cleanup(); }

private:
  // ── media-configure signal ───────────────────────────────────────────────
  static void on_media_configure_static(GstRTSPMediaFactory * /*factory*/,
                                        GstRTSPMedia *media,
                                        gpointer user_data) {
    static_cast<RTSPCameraStreamer *>(user_data)->on_media_configure(media);
  }

  void on_media_configure(GstRTSPMedia *media) {
    GstElement *bin = gst_rtsp_media_get_element(media);
    if (!bin) {
      RCLCPP_ERROR(get_logger(), "media-configure: no pipeline");
      return;
    }

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(bin), "ros_sink");
    if (!appsink) {
      RCLCPP_ERROR(get_logger(), "media-configure: ros_sink not found");
      gst_object_unref(bin);
      return;
    }

    GstAppSinkCallbacks cbs{};
    cbs.new_sample = &RTSPCameraStreamer::on_new_sample_static;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &cbs, this, nullptr);

    gst_object_unref(appsink);
    gst_object_unref(bin);

    RCLCPP_INFO(get_logger(), "RTSP client connected — ROS2 publishing active");
  }

  // ── appsink callback ─────────────────────────────────────────────────────
  static GstFlowReturn on_new_sample_static(GstAppSink *sink,
                                            gpointer user_data) {
    return static_cast<RTSPCameraStreamer *>(user_data)->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstAppSink *sink) {
    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (!sample)
      return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstStructure *st = gst_caps_get_structure(gst_sample_get_caps(sample), 0);

    int w = 0, h = 0;
    gst_structure_get_int(st, "width", &w);
    gst_structure_get_int(st, "height", &h);

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      auto stamp = now();

      // ── Image message ──────────────────────────────────────────────────
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      img_msg->header.stamp = stamp;
      img_msg->header.frame_id = frame_id_;
      img_msg->width = static_cast<uint32_t>(w);
      img_msg->height = static_cast<uint32_t>(h);
      img_msg->encoding = "rgb8";
      img_msg->is_bigendian = 0;
      img_msg->step = static_cast<uint32_t>(w * 3);
      img_msg->data.assign(map.data, map.data + map.size);
      image_pub_.publish(img_msg);

      // ── CameraInfo message ─────────────────────────────────────────────
      auto ci_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
          camera_info_manager_->getCameraInfo());
      ci_msg->header.stamp = stamp;
      ci_msg->header.frame_id = frame_id_;
      ci_msg->width = static_cast<uint32_t>(w);
      ci_msg->height = static_cast<uint32_t>(h);
      camera_info_pub_->publish(*ci_msg);

      gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // ── Cleanup ───────────────────────────────────────────────────────────────
  void cleanup() {
    if (server_) {
      g_object_unref(server_);
      server_ = nullptr;
    }
    if (loop_) {
      g_main_loop_quit(loop_);
    }
    if (glib_thread_.joinable())
      glib_thread_.join();
    if (loop_) {
      g_main_loop_unref(loop_);
      loop_ = nullptr;
    }
    if (glib_context_) {
      g_main_context_unref(glib_context_);
      glib_context_ = nullptr;
    }
  }

  // ── Members ───────────────────────────────────────────────────────────────
  GstRTSPServer *server_ = nullptr;
  GMainContext *glib_context_ = nullptr;
  GMainLoop *loop_ = nullptr;
  std::thread glib_thread_;

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::string ros_topic_basename_;
  std::string frame_id_;
};

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RTSPCameraStreamer>();
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rtsp_camera_streamer"), "Error: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}