#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/calib3d.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "custom_msgs/msg/aruco_pose.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>

class ArucoDetectorSingle {
  inline static rclcpp::Publisher<custom_msgs::msg::ArucoPose>::SharedPtr aruco_pose_publisher_ = nullptr;
  inline static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_ = nullptr;
  inline static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_ = nullptr;
public:
  ArucoDetectorSingle(rclcpp::Node::SharedPtr node,
                      const std::string &camera_label)
      : node_(node), camera_label_(camera_label), camera_info_received_(false) {

    // Create camera info subscription
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/" + camera_label_ + "/camera_info", 10, std::bind(
            &ArucoDetectorSingle::cameraInfoCallback, this, std::placeholders::_1));

    // Create image_transport subscriber
    image_transport::ImageTransport it(node_);
    image_sub_ = it.subscribe("/" + camera_label_ + "/image", 10,
        std::bind(&ArucoDetectorSingle::imageCallback, this, std::placeholders::_1));

    // Create publisher for ArucoPose messages on /vision/aruco
    if (aruco_pose_publisher_ == nullptr) {
      aruco_pose_publisher_ = node_->create_publisher<custom_msgs::msg::ArucoPose>(
        "/vision/aruco", 10);
      }

    // Create publisher for debug visualization image
    if (debug_image_publisher_ == nullptr) {
      debug_image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "/vision/aruco/image", 10);
    }

    if (tf_br_ == nullptr) {
      tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "ArUco detector initialized for camera %s, waiting for camera info...",
        camera_label_.c_str());
  }

  // Shared resources - static members

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
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;

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
        auto tf_br = tf_br_;
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

          // Publish ArucoPose message
          custom_msgs::msg::ArucoPose aruco_msg;
          aruco_msg.pose.header.stamp = node_->now();
          aruco_msg.pose.header.frame_id = cameraName;
          aruco_msg.pose.pose.position.x = tf_orig.getOrigin().x();
          aruco_msg.pose.pose.position.y = tf_orig.getOrigin().y();
          aruco_msg.pose.pose.position.z = tf_orig.getOrigin().z();
          aruco_msg.pose.pose.orientation.x = tf_orig.getRotation().x();
          aruco_msg.pose.pose.orientation.y = tf_orig.getRotation().y();
          aruco_msg.pose.pose.orientation.z = tf_orig.getRotation().z();
          aruco_msg.pose.pose.orientation.w = tf_orig.getRotation().w();
          aruco_msg.marker_id = markerIds_[i];
          
          aruco_pose_publisher_->publish(aruco_msg);

          // Draw coordinate axes on the debug image
          cv::drawFrameAxes(imageCopy, cameraMatrix_, distCoeffs_, rvecs_[i],
                            tvecs_[i], getMarkerLength() * 1.5f, 2);
        }

        // Draw detected markers on the debug image
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners_, markerIds_);
      }

      // Publish debug visualization image
      auto debug_image = cv_bridge::CvImage(msg->header, "bgr8", imageCopy).toImageMsg();
      debug_image->header.frame_id = cameraName; // Set frame_id for visualization
      debug_image_publisher_->publish(*debug_image);

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
      "camera_labels", std::vector<std::string>{"camera_front", "camera_bottom"});

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