#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/calib3d.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>

class ArucoDetectorSingle {
public:
  ArucoDetectorSingle(rclcpp::Node::SharedPtr node,
                      const std::string &camera_label)
      : node_(node), camera_label_(camera_label), camera_info_received_(false) {

    // Create camera info subscription
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/" + camera_label_ + "/camera_info", 10, std::bind(
            &ArucoDetectorSingle::cameraInfoCallback, this, std::placeholders::_1));

    // Create image subscription
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/" + camera_label_ + "/image_raw", 10, std::bind(
            &ArucoDetectorSingle::imageCallback, this, std::placeholders::_1));

    // Create publisher for detected markers visualization
    marker_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/" + camera_label_ + "/image_detected_markers", 10);

    // Publisher for detected marker IDs
    marker_id_publisher_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/" + camera_label_ + "/marker_ids", 10);

    RCLCPP_INFO(
        node_->get_logger(),
        "ArUco detector initialized for camera %s, waiting for camera info...",
        camera_label_.c_str());
  }

  // Shared resources - static members
  static std::shared_ptr<tf2_ros::TransformBroadcaster>
  getTfBroadcaster(rclcpp::Node::SharedPtr node) {
    static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br = nullptr;
    if (!tf_br) {
      tf_br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }
    return tf_br;
  }

  static cv::Ptr<cv::aruco::Dictionary> getDictionary() {
    static cv::Ptr<cv::aruco::Dictionary> dictionary = nullptr;
    if (!dictionary) {
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    }
    return dictionary;
  }

  static const cv::Mat &getObjectPoints() {
    static cv::Mat objPoints;
    static bool initialized = false;
    if (!initialized) {
      const float markerLength = 0.15f;
      objPoints = cv::Mat(4, 1, CV_32FC3);
      objPoints.ptr<cv::Vec3f>(0)[0] =
          cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
      objPoints.ptr<cv::Vec3f>(0)[1] =
          cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
      objPoints.ptr<cv::Vec3f>(0)[2] =
          cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
      objPoints.ptr<cv::Vec3f>(0)[3] =
          cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
      initialized = true;
    }
    return objPoints;
  }

  static constexpr float getMarkerLength() { return 0.15f; }
  static constexpr int getMaxMarkerId() { return 27; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string camera_label_;

  // Camera calibration
  cv::Mat cameraMatrix_, distCoeffs_;
  bool camera_info_received_ = false;

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      marker_id_publisher_;

  // Per-camera detection results (not shared to avoid thread safety issues)
  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_;
  std::vector<cv::Vec3d> rvecs_;
  std::vector<cv::Vec3d> tvecs_;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // Convert ROS camera info to OpenCV format
    cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix_.at<double>(0, 0) = msg->k[0]; // fx
    cameraMatrix_.at<double>(1, 1) = msg->k[4]; // fy
    cameraMatrix_.at<double>(0, 2) = msg->k[2]; // cx
    cameraMatrix_.at<double>(1, 2) = msg->k[5]; // cy

    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    for (int i = 0; i < std::min(5, (int)msg->d.size()); i++) {
      distCoeffs_.at<double>(0, i) = msg->d[i];
    }

    if (!camera_info_received_) {
      RCLCPP_INFO(node_->get_logger(), "Received camera info for %s",
                  camera_label_.c_str());
      camera_info_received_ = true;
    }
  }

  static void quaternionFromRvecs(const cv::Vec3d &rvec, tf2::Quaternion &q) {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    tf2::Matrix3x3 tf_rotation(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));

    tf_rotation.getRotation(q);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    // Skip processing if we haven't received camera info yet
    if (!camera_info_received_) {
      return;
    }

    const std::string cameraName = camera_label_;
    cv::Mat imageCopy;
    // std::vector<int> detected_ids;

    try {
      // Detect ArUco markers using shared dictionary
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::aruco::detectMarkers(image, getDictionary(), markerCorners_,
                               markerIds_);

      image.copyTo(imageCopy);

      if (!markerIds_.empty()) {
        // Resize vectors to match number of detected markers
        rvecs_.resize(markerIds_.size());
        tvecs_.resize(markerIds_.size());

        // Get shared resources
        auto tf_br = getTfBroadcaster(node_);
        const cv::Mat &objPoints = getObjectPoints();

        // Process each detected marker
        for (size_t i = 0; i < markerIds_.size(); ++i) {
          if (markerIds_[i] > getMaxMarkerId()) {
            continue;
          }

          // Solve PnP to get marker pose using shared object points
          cv::solvePnP(objPoints, markerCorners_[i], cameraMatrix_, distCoeffs_,
                       rvecs_[i], tvecs_[i]);
          cv::solvePnPRefineLM(objPoints, markerCorners_[i], cameraMatrix_,
                               distCoeffs_, rvecs_[i], tvecs_[i]);

          // Create transform message
          geometry_msgs::msg::TransformStamped transform;
          transform.header.stamp = node_->now();
          transform.child_frame_id = cv::format(
              "aruco_marker_%d_from_%s", markerIds_[i], camera_label_.c_str());
          transform.header.frame_id = cameraName;

          // Convert pose to transform
          tf2::Vector3 t_orig(tvecs_[i][0], tvecs_[i][1], tvecs_[i][2]);
          tf2::Quaternion q_orig;
          quaternionFromRvecs(rvecs_[i], q_orig);
          q_orig.normalize();

          tf2::Transform tf_orig(q_orig, t_orig);

          transform.transform.translation.x = tf_orig.getOrigin().x();
          transform.transform.translation.y = tf_orig.getOrigin().y();
          transform.transform.translation.z = tf_orig.getOrigin().z();
          transform.transform.rotation.x = tf_orig.getRotation().x();
          transform.transform.rotation.y = tf_orig.getRotation().y();
          transform.transform.rotation.z = tf_orig.getRotation().z();
          transform.transform.rotation.w = tf_orig.getRotation().w();

          // Broadcast transform using shared broadcaster
          tf_br->sendTransform(transform);

          // Draw coordinate axes on the marker
          cv::drawFrameAxes(imageCopy, cameraMatrix_, distCoeffs_, rvecs_[i],
                            tvecs_[i], getMarkerLength() * 1.5f, 2);
        }

        // Draw detected markers
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners_, markerIds_);
      }

      // Publish visualization image
      marker_publisher_->publish(
          *cv_bridge::CvImage(msg->header, "bgr8", imageCopy).toImageMsg());

      marker_id_publisher_->publish([this]() {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data = markerIds_;
        return msg;
      }());

    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Error during image processing: %s",
                   e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create a single shared node
  auto node = std::make_shared<rclcpp::Node>("aruco_detector");

  // Get camera labels from parameter
  std::vector<std::string> camera_labels = node->declare_parameter(
      "camera_labels", std::vector<std::string>{"camera_1"});

  RCLCPP_INFO(node->get_logger(), "Starting ArUco detector for %zu cameras",
              camera_labels.size());

  // Create detector instances for each camera
  std::vector<std::unique_ptr<ArucoDetectorSingle>> detectors;
  detectors.reserve(camera_labels.size());
  for (const auto &camera_label : camera_labels) {
    detectors.push_back(
        std::make_unique<ArucoDetectorSingle>(node, camera_label));
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}