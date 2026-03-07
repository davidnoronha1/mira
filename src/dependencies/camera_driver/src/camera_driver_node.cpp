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

#include "gst/gstdevicemonitor.h"
#include "gst/gststructure.h"
#include "gst/rtsp-server/rtsp-server-object.h"
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

const char *get_machine_ip() {
  auto x = getenv("MACHINE_IP");
  return x == nullptr ? "{MACHINE_IP}" : x;
}

class RTSPCameraStreamer : public rclcpp::Node {
public:
  RTSPCameraStreamer() : Node("rtsp_camera_streamer") {}

  // ── Encoder detection ─────────────────────────────────────────────────────
  enum class EncoderType { X264, QSV, NVENC, V4L2 };

  struct EncoderInfo {
    EncoderType type;
    std::string name;        // GStreamer element name
    std::string extra_props; // properties appended after bitrate
  };

  static std::string caps_for_encoder(EncoderType t) {
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

  static bool gst_element_exists(const std::string &name) {
    GstElementFactory *f = gst_element_factory_find(name.c_str());
    if (!f)
      return false;
    gst_object_unref(f);
    return true;
  }

  EncoderInfo detect_best_encoder(long bitrate, long framerate) {
    // ── Intel QSV ──────────────────────────────────────────────────────────
    if (gst_element_exists("qsvh264enc")) {
      RCLCPP_INFO(get_logger(), "Encoder: Intel QSV (qsvh264enc)");
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
      RCLCPP_INFO(get_logger(), "Encoder: NVIDIA NVENC (nvh264enc)");
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
      RCLCPP_INFO(get_logger(), "Encoder: V4L2 M2M hardware (v4l2h264enc)");
      return {EncoderType::V4L2, "v4l2h264enc",
              " extra-controls=\"encode,video_bitrate=" +
                  std::to_string(bitrate * 1000) + "\""};
    }

    // ── Software x264 fallback ─────────────────────────────────────────────
    RCLCPP_WARN(get_logger(),
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

  static bool usb_port_matches(const gchar *prop_value,
                               const std::string &usb_port) {
    if (!prop_value)
      return false;
    return std::string(prop_value).find(usb_port) != std::string::npos;
  }

  // Find a camera whose physical USB port matches `usb_port`.
  GstDevice *find_camera_by_usb_port(const std::string &usb_port,
                                     std::string &device_path) {
    GList *devices = nullptr;
    GstDeviceMonitor *monitor = gst_device_monitor_new();
    gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
    if (!monitor)
      return nullptr;

    GstDevice *result = nullptr;
    devices = gst_device_monitor_get_devices(monitor);

    for (GList *l = devices; l && !result; l = l->next) {
      GstDevice *dev = GST_DEVICE(l->data);
      GstStructure *props = gst_device_get_properties(dev);
      if (!props)
        continue;

      {
        gchar *props_str = gst_structure_to_string(props);
        RCLCPP_DEBUG(get_logger(), "Device properties: %s", props_str);
        g_free(props_str);
      }

      const gchar *bus_path =
          gst_structure_get_string(props, "device.bus_path");
      const gchar *sysfs = gst_structure_get_string(props, "sysfs.path");
      const gchar *obj_path = gst_structure_get_string(props, "v4l2.device.bus_info");

      bool matched = usb_port_matches(bus_path, usb_port) ||
                     usb_port_matches(sysfs, usb_port) ||
                     usb_port_matches(obj_path, usb_port);

      if (matched) {
        RCLCPP_INFO(get_logger(), "USB port match on device: %s  (bus_path=%s)",
                    gst_device_get_display_name(dev),
                    bus_path ? bus_path : "<none>");

        const gchar *v4l2_path =
            gst_structure_get_string(props, "api.v4l2.path");
        if (!v4l2_path)
          v4l2_path = gst_structure_get_string(props, "device.path");
        if (v4l2_path && device_path.empty()) {
          device_path = v4l2_path;
          RCLCPP_INFO(get_logger(), "Resolved USB port %s → device path %s",
                      usb_port.c_str(), device_path.c_str());
        }

        result = GST_DEVICE(gst_object_ref(dev));
      }

      gst_structure_free(props);
    }

    if (!result)
      RCLCPP_WARN(get_logger(),
                  "No device found at USB port '%s'. "
                  "Run `gst-device-monitor-1.0 Video/Source` and look for "
                  "device.bus_path or sysfs-path to find the correct value.",
                  usb_port.c_str());

    g_list_free_full(devices, gst_object_unref);
    gst_device_monitor_stop(monitor);
    gst_object_unref(monitor);
    return result;
  }

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
    target_device = find_camera_by_usb_port(usb_port, device_path);

    if (target_device == nullptr) {
      RCLCPP_INFO(
          get_logger(),
          "Trying to find camera by vendor (%x), product (%x), serial (%s)",
          vendor, product, serial.c_str());
      target_device = find_camera_by_id(vendor, product, serial);
    }

    if (target_device == nullptr) {
      RCLCPP_INFO(get_logger(),
                  "Previous failed! Trying to find device by path (%s)",
                  device_path.c_str());
      target_device = find_camera_by_path(device_path);
    }

    if (target_device == nullptr) {
      RCLCPP_INFO(get_logger(), "Previous failed! Trying to find any device!");
      target_device = find_camera_any(device_path);
    }

    RCLCPP_INFO(get_logger(), "Using webcam: %s",
                target_device != nullptr
                    ? gst_device_get_display_name(target_device)
                    : device_path.c_str());

    // ── Capability negotiation ───────────────────────────────────────────────
    std::set<std::string> supported_formats;
    std::set<std::pair<int, int>> supported_resolutions;
    query_camera_capabilities(target_device, supported_formats,
                              supported_resolutions);

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
                "-framedrop -vf 'setpts=0' rtsp://%s:%d/%s",
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
                                              framerate, bitrate);
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
  // ── Pipeline builder ─────────────────────────────────────────────────────
  std::string build_pipeline(const std::string &fmt,
                             const std::string &device_path,
                             long width, long height,
                             long framerate, long bitrate) {
    std::string gst_fmt = gst_caps_for_format(fmt);
    std::ostringstream ss;

    if (strcasecmp(fmt.c_str(), "H264") == 0) {
      // ── H.264 passthrough: camera already encodes, skip re-encode ─────────
      // RTSP branch gets the raw H.264 bitstream directly.
      // ROS2 branch decodes to RGB for image messages.
      RCLCPP_INFO(get_logger(),
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
      auto best_enc = detect_best_encoder(bitrate, framerate);

      if (strcasecmp(fmt.c_str(), "MJPEG") == 0) {
        RCLCPP_INFO(get_logger(), "Pipeline mode: MJPEG → decode → x264enc");
      } else {
        RCLCPP_INFO(get_logger(), "Pipeline mode: raw → videoconvert → x264enc");
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

  // ── Device discovery ─────────────────────────────────────────────────────
  GstDevice *find_camera_by_id(long vendor, long product,
                               const std::string &serial) {
    if (vendor == -1 || product == -1 || serial == "") {
      RCLCPP_WARN(get_logger(),
                  "Invalid product, vendor, serial recieved, failing!");
      return nullptr;
    }
    GstDeviceMonitor *monitor = gst_device_monitor_new();
    gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
    if (!gst_device_monitor_start(monitor)) {
      RCLCPP_WARN(get_logger(), "Failed to start device monitor");
      gst_object_unref(monitor);
      return nullptr;
    }

    GList *devices = gst_device_monitor_get_devices(monitor);
    GstDevice *result = nullptr;

    for (GList *l = devices; l && !result; l = l->next) {
      GstDevice *dev = GST_DEVICE(l->data);
      GstStructure *props = gst_device_get_properties(dev);
      if (props == nullptr)
        continue;

      const gchar *dev_vendor_str =
          gst_structure_get_string(props, "device.vendor.id");
      const gchar *dev_product_str =
          gst_structure_get_string(props, "device.product.id");
      const gchar *dev_sn = gst_structure_get_string(props, "device.serial");

      if (dev_product_str == nullptr || dev_vendor_str == nullptr ||
          dev_sn == nullptr) {
        RCLCPP_WARN(get_logger(),
                    "Device has no device.vendor.id, device.product.id or "
                    "device.serial property, Check gst-device-monitor-1.0");
        continue;
      }

      int dev_vendor = strtol(dev_vendor_str, nullptr, 16);
      int dev_product = strtol(dev_product_str, nullptr, 16);

      RCLCPP_INFO(get_logger(), "Found device (%x, %x, %s)", dev_vendor,
                  dev_product, dev_sn);

      bool match =
          (vendor == 0 || dev_vendor == (guint)vendor) &&
          (product == 0 || dev_product == (guint)product) &&
          (serial.empty() ||
           (dev_sn && std::string(dev_sn).find(serial) != std::string::npos));

      if (match)
        result = GST_DEVICE(gst_object_ref(dev));
      gst_structure_free(props);
    }

    g_list_free_full(devices, gst_object_unref);
    gst_device_monitor_stop(monitor);
    gst_object_unref(monitor);
    return result;
  }

  GstDevice *find_camera_any(std::string &device_path) {
    GstDeviceMonitor *monitor = gst_device_monitor_new();
    gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
    if (!gst_device_monitor_start(monitor)) {
      RCLCPP_WARN(get_logger(), "Failed to start device monitor");
      gst_object_unref(monitor);
      return nullptr;
    }

    GList *devices = gst_device_monitor_get_devices(monitor);
  retry:
    GstDevice *result = GST_DEVICE(g_object_ref(g_list_first(devices)->data));
    auto props = gst_device_get_properties(result);
    auto dev_path = gst_structure_get_string(props, "api.v4l2.path");
    if (dev_path == nullptr) {
      dev_path = gst_structure_get_string(props, "device.path");
    }
    if (dev_path != nullptr) {
      RCLCPP_INFO(get_logger(), "Got device with path: %s", dev_path);
      device_path = g_strdup(dev_path);
      gst_structure_free(props);
    } else {
      devices = g_list_first(devices);
      gst_structure_free(props);
      goto retry;
    }

    g_list_free_full(devices, gst_object_unref);
    gst_device_monitor_stop(monitor);
    gst_object_unref(monitor);
    return result;
  }

  GstDevice *find_camera_by_path(std::string &device_path) {
    if (device_path == "") {
      RCLCPP_WARN(get_logger(), "No device path provided, Failing!");
      return nullptr;
    }
    GstDeviceMonitor *monitor = gst_device_monitor_new();
    gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
    if (!gst_device_monitor_start(monitor)) {
      RCLCPP_WARN(get_logger(), "Failed to start device monitor");
      gst_object_unref(monitor);
      return nullptr;
    }

    GList *devices = gst_device_monitor_get_devices(monitor);
    GstDevice *result = nullptr;

    for (GList *l = devices; l && result != nullptr; l = l->next) {
      GstDevice *dev = GST_DEVICE(l->data);
      GstStructure *props = gst_device_get_properties(dev);
      if (props == nullptr)
        continue;

      const gchar *dev_path = gst_structure_get_string(props, "api.v4l2.path");

      if (dev_path == device_path) {
        result = g_object_ref(dev);
      }

      gst_structure_free(props);
    }

    g_list_free_full(devices, gst_object_unref);
    gst_device_monitor_stop(monitor);
    gst_object_unref(monitor);
    return result;
  }

  // ── Capability query ─────────────────────────────────────────────────────
  void query_camera_capabilities(GstDevice *dev, std::set<std::string> &formats,
                                 std::set<std::pair<int, int>> &resolutions) {
    GstCaps *caps = gst_device_get_caps(dev);
    if (!caps)
      return;

    for (guint i = 0; i < gst_caps_get_size(caps); i++) {
      GstStructure *s = gst_caps_get_structure(caps, i);
      const gchar *mtype = gst_structure_get_name(s);

      if (g_strcmp0(mtype, "image/jpeg") == 0) {
        formats.insert("MJPEG");
      } else if (g_strcmp0(mtype, "video/x-h264") == 0) {
        // H.264 compressed output — passthrough capable
        formats.insert("H264");
      } else if (g_strcmp0(mtype, "video/x-raw") == 0) {
        const gchar *f = gst_structure_get_string(s, "format");
        if (g_strcmp0(f, "YUY2") == 0)
          formats.insert("YUYV");
        else if (g_strcmp0(f, "NV12") == 0)
          formats.insert("NV12");
        else if (g_strcmp0(f, "RGB") == 0)
          formats.insert("RGB");
        else if (g_strcmp0(f, "BGR") == 0)
          formats.insert("BGR");
      }

      int w = 0, h = 0;
      if (gst_structure_get_int(s, "width", &w) &&
          gst_structure_get_int(s, "height", &h))
        resolutions.insert({w, h});
    }

    if (!formats.empty()) {
      std::ostringstream os;
      for (auto &f : formats)
        os << f << " ";
      RCLCPP_INFO(get_logger(), "Supported formats: %s", os.str().c_str());
    }
    if (!resolutions.empty()) {
      std::ostringstream os;
      for (auto &r : resolutions)
        os << r.first << "x" << r.second << " ";
      RCLCPP_INFO(get_logger(), "Supported resolutions: %s", os.str().c_str());
    }

    gst_caps_unref(caps);
  }

  // ── Helpers ───────────────────────────────────────────────────────────────
  static std::string gst_caps_for_format(const std::string &fmt) {
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
