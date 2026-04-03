#include "device_finding.hpp"
#include <gst/gstdevicemonitor.h>
#include <gst/gststructure.h>
#include <sstream>

bool usb_port_matches(const gchar *prop_value, const std::string &usb_port) {
  if (!prop_value)
    return false;
  return std::string(prop_value).find(usb_port) != std::string::npos;
}

GstDevice *find_camera_by_usb_port(const std::string &usb_port,
                                   std::string &device_path,
                                   rclcpp::Logger logger) {
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
      RCLCPP_DEBUG(logger, "Device properties: %s", props_str);
      g_free(props_str);
    }

    const gchar *bus_path = gst_structure_get_string(props, "device.bus_path");
    const gchar *sysfs = gst_structure_get_string(props, "sysfs.path");
    const gchar *obj_path = gst_structure_get_string(props, "v4l2.device.bus_info");

    bool matched = usb_port_matches(bus_path, usb_port) ||
                   usb_port_matches(sysfs, usb_port) ||
                   usb_port_matches(obj_path, usb_port);

    if (matched) {
      RCLCPP_INFO(logger, "USB port match on device: %s  (bus_path=%s)",
                  gst_device_get_display_name(dev),
                  bus_path ? bus_path : "<none>");

      const gchar *v4l2_path = gst_structure_get_string(props, "api.v4l2.path");
      if (!v4l2_path)
        v4l2_path = gst_structure_get_string(props, "device.path");
      if (v4l2_path && device_path.empty()) {
        device_path = v4l2_path;
        RCLCPP_INFO(logger, "Resolved USB port %s → device path %s",
                    usb_port.c_str(), device_path.c_str());
      }

      result = GST_DEVICE(gst_object_ref(dev));
    }

    gst_structure_free(props);
  }

  if (!result)
    RCLCPP_WARN(logger,
                "No device found at USB port '%s'. "
                "Run `gst-device-monitor-1.0 Video/Source` and look for "
                "device.bus_path or sysfs-path to find the correct value.",
                usb_port.c_str());

  g_list_free_full(devices, gst_object_unref);
  gst_device_monitor_stop(monitor);
  gst_object_unref(monitor);
  return result;
}

GstDevice *find_camera_by_id(long vendor, long product,
                             const std::string &serial, rclcpp::Logger logger) {
  if (vendor == -1 || product == -1 || serial == "") {
    RCLCPP_WARN(logger, "Invalid product, vendor, serial recieved, failing!");
    return nullptr;
  }
  GstDeviceMonitor *monitor = gst_device_monitor_new();
  gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
  if (!gst_device_monitor_start(monitor)) {
    RCLCPP_WARN(logger, "Failed to start device monitor");
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
      RCLCPP_WARN(logger,
                  "Device has no device.vendor.id, device.product.id or "
                  "device.serial property, Check gst-device-monitor-1.0");
      continue;
    }

    int dev_vendor = strtol(dev_vendor_str, nullptr, 16);
    int dev_product = strtol(dev_product_str, nullptr, 16);

    RCLCPP_INFO(logger, "Found device (%x, %x, %s)", dev_vendor, dev_product,
                dev_sn);

    bool match =
        (vendor == 0 || (long)dev_vendor == vendor) &&
        (product == 0 || (long)dev_product == product) &&
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

GstDevice *find_camera_any(std::string &device_path, rclcpp::Logger logger) {
  GstDeviceMonitor *monitor = gst_device_monitor_new();
  gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
  if (!gst_device_monitor_start(monitor)) {
    RCLCPP_WARN(logger, "Failed to start device monitor");
    gst_object_unref(monitor);
    return nullptr;
  }

  GList *devices = gst_device_monitor_get_devices(monitor);
retry:
  if (!devices) {
    RCLCPP_WARN(logger, "No video devices found at all");
    gst_device_monitor_stop(monitor);
    gst_object_unref(monitor);
    return nullptr;
  }
  GstDevice *result = GST_DEVICE(g_object_ref(g_list_first(devices)->data));
  auto props = gst_device_get_properties(result);
  auto dev_path = gst_structure_get_string(props, "api.v4l2.path");
  if (dev_path == nullptr) {
    dev_path = gst_structure_get_string(props, "device.path");
  }
  if (dev_path != nullptr) {
    RCLCPP_INFO(logger, "Got device with path: %s", dev_path);
    device_path = g_strdup(dev_path);
    gst_structure_free(props);
  } else {
    devices = g_list_next(devices);
    gst_structure_free(props);
    goto retry;
  }

  g_list_free_full(devices, gst_object_unref);
  gst_device_monitor_stop(monitor);
  gst_object_unref(monitor);
  return result;
}

GstDevice *find_camera_by_path(std::string &device_path, rclcpp::Logger logger) {
  if (device_path == "") {
    RCLCPP_WARN(logger, "No device path provided, Failing!");
    return nullptr;
  }
  GstDeviceMonitor *monitor = gst_device_monitor_new();
  gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);
  if (!gst_device_monitor_start(monitor)) {
    RCLCPP_WARN(logger, "Failed to start device monitor");
    gst_object_unref(monitor);
    return nullptr;
  }

  GList *devices = gst_device_monitor_get_devices(monitor);
  GstDevice *result = nullptr;

  for (GList *l = devices; l && result == nullptr; l = l->next) {
    GstDevice *dev = GST_DEVICE(l->data);
    GstStructure *props = gst_device_get_properties(dev);
    if (props == nullptr)
      continue;

    const gchar *dev_path = gst_structure_get_string(props, "api.v4l2.path");
    if (!dev_path)
      dev_path = gst_structure_get_string(props, "device.path");

    if (dev_path && device_path == dev_path) {
      result = GST_DEVICE(g_object_ref(dev));
    }

    gst_structure_free(props);
  }

  g_list_free_full(devices, gst_object_unref);
  gst_device_monitor_stop(monitor);
  gst_object_unref(monitor);
  return result;
}

void query_camera_capabilities(GstDevice *dev, std::set<std::string> &formats,
                               std::set<std::pair<int, int>> &resolutions,
                               rclcpp::Logger logger) {
  if (!dev) return;
  GstCaps *caps = gst_device_get_caps(dev);
  if (!caps)
    return;

  for (guint i = 0; i < gst_caps_get_size(caps); i++) {
    GstStructure *s = gst_caps_get_structure(caps, i);
    const gchar *mtype = gst_structure_get_name(s);

    if (g_strcmp0(mtype, "image/jpeg") == 0) {
      formats.insert("MJPEG");
    } else if (g_strcmp0(mtype, "video/x-h264") == 0) {
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
    RCLCPP_INFO(logger, "Supported formats: %s", os.str().c_str());
  }
  if (!resolutions.empty()) {
    std::ostringstream os;
    for (auto &r : resolutions)
      os << r.first << "x" << r.second << " ";
    RCLCPP_INFO(logger, "Supported resolutions: %s", os.str().c_str());
  }

  gst_caps_unref(caps);
}