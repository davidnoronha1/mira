
// camera_rtsp_streamer_v6.cpp
//
// Single-pipeline architecture:
//
//   v4l2src → decode → videoconvert → tee ┬→ queue → x264enc → rtph264pay (RTSP
//   clients)
//                                          └→ queue → videoconvert → RGB →
//                                          appsink  (ROS2)
//
// The RTSP media factory owns and runs the pipeline. We hook the factory's
// "media-configure" signal to retrieve the appsink from inside that pipeline
// and attach our ROS2 callback — so the camera is opened exactly once and
// encoding happens exactly once regardless of the number of RTSP clients.

#include "glib.h"
#include "gst/gstdevicemonitor.h"
#include "gst/gststructure.h"
#include "gst/rtsp-server/rtsp-server-object.h"
#include <future>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/gstdevice.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
                  " target-usage=7" // 1=quality … 7=speed (lowest latency)
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
                  " preset=low-latency-hp" // hp = high performance
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
  //
  // GStreamer / udev exposes the port in several properties depending on the
  // stack in use:
  //   • "device.bus_path"  – most common (udev), e.g. "usb-0000:00:14.0-1.2"
  //   • "sysfs-path"       – full sysfs path, contains the bus-port string
  //   • "api.v4l2.path"    – PipeWire/WirePlumber (e.g. "/dev/video2"); not
  //                          a USB port, but we fall through to path matching
  //                          below if nothing else matched.
  //
  // If a matching device is found its /dev/videoN path is written into
  // `device_path` so the pipeline string is correct even if the caller did not
  // supply one.
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

      // Log all properties at DEBUG level so users can figure out what their
      // system exposes (handy when usb_port does not match anything).
      {
        gchar *props_str = gst_structure_to_string(props);
        RCLCPP_DEBUG(get_logger(), "Device properties: %s", props_str);
        g_free(props_str);
      }

      // Check the three property keys that may carry USB topology info.
      const gchar *bus_path =
          gst_structure_get_string(props, "device.bus_path");
      const gchar *sysfs = gst_structure_get_string(props, "sysfs-path");
      // PipeWire sometimes stores it under object.path, e.g. "v4l2:/dev/video2"
      const gchar *obj_path = gst_structure_get_string(props, "object.path");

      bool matched = usb_port_matches(bus_path, usb_port) ||
                     usb_port_matches(sysfs, usb_port) ||
                     usb_port_matches(obj_path, usb_port);

      if (matched) {
        RCLCPP_INFO(get_logger(), "USB port match on device: %s  (bus_path=%s)",
                    gst_device_get_display_name(dev),
                    bus_path ? bus_path : "<none>");

        // Resolve the /dev/videoN path so the pipeline can use it.
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
    // declare_parameter("camera")

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

    frame_id_ = get_parameter("camera_frame_id").as_string();
    ros_topic_ = get_parameter("ros_topic").as_string();
    if (ros_topic_ == "") {
      ros_topic_ = frame_id_;
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
    RCLCPP_INFO(get_logger(), "ROS2 Image Topic:   %s", ros_topic_.c_str());
    RCLCPP_INFO(get_logger(),
                "Command to receive: ffplay -fflags nobuffer -flags low_delay "
                "-framedrop -vf 'setpts=0' rtsp://%s:%d/%s",
                machine_ip, port, mount_point);
    RCLCPP_INFO(get_logger(),
                "============================================================");

    // ── image_transport publisher ────────────────────────────────────────────
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise(ros_topic_, 1);

    // ── Build the shared pipeline string ────────────────────────────────────
    std::string gst_fmt = gst_caps_for_format(fmt);
    auto best_enc = detect_best_encoder(bitrate, framerate);
    std::ostringstream ss;
    ss << "( " << " v4l2src device=" << device_path << " ! " << gst_fmt
       << ",width=" << width << ",height=" << height
       << ",framerate=" << framerate << "/1 ! ";

    if (strcasecmp(fmt.c_str(), "MJPEG") == 0)
      ss << "jpegdec ! ";

    ss << " tee name=t "

       // Branch 1 – RTSP/H.264
       << "t. ! queue leaky=downstream max-size-buffers=2 ! "
       //  << "x264enc bitrate=" << bitrate
       //  << " speed-preset=ultrafast tune=zerolatency"
       //  << " key-int-max=" << framerate
       //  << " bframes=0 sliced-threads=true rc-lookahead=0 sync-lookahead=0 "
       //     "vbv-buf-capacity=0 ! "
       << " videoconvert ! " << caps_for_encoder(best_enc.type) << " ! "
       << best_enc.name << " " << best_enc.extra_props << " ! "
       << "rtph264pay name=pay0 pt=96 config-interval=1 "

       // Branch 2 – ROS2 appsink (RGB, no extra encoding)
       << "t. ! queue leaky=downstream max-size-buffers=2 ! "
       << "videoconvert ! video/x-raw,format=RGB ! "
       << "appsink name=ros_sink max-buffers=2 drop=true sync=false "
       << ")";

    std::string pipeline_str = ss.str();
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
    // Use a dedicated context so we can guarantee the loop is running before
    // calling attach() — otherwise attach() returns 0 on a quiet context.
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
    loop_ready_fut.wait(); // blocks until the loop is actually spinning

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
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      msg->header.frame_id = frame_id_;
      msg->width = static_cast<uint32_t>(w);
      msg->height = static_cast<uint32_t>(h);
      msg->encoding = "rgb8";
      msg->is_bigendian = 0;
      msg->step = static_cast<uint32_t>(w * 3);
      msg->data.assign(map.data, map.data + map.size);
      image_pub_.publish(msg);
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
    if (strcasecmp(fmt.c_str(), "RGB") == 0)
      return "video/x-raw,format=RGB";
    if (strcasecmp(fmt.c_str(), "BGR") == 0)
      return "video/x-raw,format=BGR";
    if (strcasecmp(fmt.c_str(), "NV12") == 0)
      return "video/x-raw,format=NV12";
    return "image/jpeg";
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
  std::string ros_topic_;
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
