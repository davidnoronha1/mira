#pragma once
#include <string>

namespace utils {

enum class ImageScheme {
    FILE,    ///< file:///path/to/video.mp4 or still image
    RTSP,    ///< rtsp://host:port/path
    RTSPS,   ///< rtsps://host:port/path  (RTSP over TLS)
    ROS2,    ///< ros2://topic/name
    CAMERA,  ///< camera://N  (webcam by integer index)
    USB,     ///< usb://  or  usb:///dev/videoN  (physical USB/V4L2 camera)
    UNKNOWN
};

inline ImageScheme parse_image_scheme(const std::string & uri)
{
    if (uri.rfind("file://",   0) == 0) return ImageScheme::FILE;
    if (uri.rfind("rtsps://",  0) == 0) return ImageScheme::RTSPS;  // must come before rtsp://
    if (uri.rfind("rtsp://",   0) == 0) return ImageScheme::RTSP;
    if (uri.rfind("ros2://",   0) == 0) return ImageScheme::ROS2;
    if (uri.rfind("camera://", 0) == 0) return ImageScheme::CAMERA;
    if (uri.rfind("usb://",    0) == 0) return ImageScheme::USB;
    return ImageScheme::UNKNOWN;
}

/// Strip the scheme prefix (everything up to and including "://").
inline std::string strip_image_scheme(const std::string & uri)
{
    auto pos = uri.find("://");
    if (pos == std::string::npos) return uri;
    return uri.substr(pos + 3);
}

/**
 * URI-aware image source base class.
 *
 * Stores the raw URI and its parsed scheme.  Subclasses implement
 * source-specific behaviour — e.g. an OpenCV-based grab() loop, a GStreamer
 * pipeline builder, or a ROS 2 subscription wrapper.
 *
 * Supported URI schemes (base class):
 *   file:///abs/path.mp4   — local video file or still image
 *   rtsp://host:port/path  — RTSP stream
 *   rtsps://host:port/path — RTSP over TLS
 *   ros2://topic/name      — ROS 2 sensor_msgs/Image topic
 *   camera://0             — webcam by index
 *   usb://                 — physical USB/V4L2 camera (extended by subclasses)
 */
class ImageSource
{
public:
    explicit ImageSource(const std::string & uri)
    : uri_(uri), scheme_(parse_image_scheme(uri)) {}

    virtual ~ImageSource() = default;

    const std::string & uri()    const { return uri_; }
    ImageScheme          scheme() const { return scheme_; }

    /// URI with the scheme prefix stripped.
    std::string path() const { return strip_image_scheme(uri_); }

    bool is_file()   const { return scheme_ == ImageScheme::FILE; }
    bool is_rtsp()   const { return scheme_ == ImageScheme::RTSP || scheme_ == ImageScheme::RTSPS; }
    bool is_ros2()   const { return scheme_ == ImageScheme::ROS2; }
    bool is_camera() const { return scheme_ == ImageScheme::CAMERA; }
    bool is_usb()    const { return scheme_ == ImageScheme::USB; }

private:
    std::string uri_;
    ImageScheme scheme_;
};

}  // namespace utils
