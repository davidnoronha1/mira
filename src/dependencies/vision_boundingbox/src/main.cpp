#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <openvino/openvino.hpp>

#if BACKEND == OPENVINO
#include "backends/openvino/yolo_v11_openvino.hpp"
#endif

using namespace std::chrono_literals;

void runWebcamMode(std::shared_ptr<rclcpp::Node> node, Yolo11OpenVinoModel& model, int camera_index) {
  cv::VideoCapture cap(camera_index);
  if (!cap.isOpened()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open camera with index %d", camera_index);
    return;
  }

  auto image_width = node->declare_parameter("image_width", 640);
  auto image_height = node->declare_parameter("image_height", 480);
  
  cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

  RCLCPP_INFO(node->get_logger(), "Starting camera capture on index %d", camera_index);

  int frame_count = 0;
  double fps = 0.0;
  auto last_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to read frame from camera");
      break;
    }

    // Process frame and get detections
    std::vector<Detection> detections;
    model.setCallback([&detections](const std::vector<Detection>& dets) {
      detections = dets;
    });
    
    model.processImage(frame);
    
    // Draw detections on frame
    cv::Mat result = drawDetections(frame, detections);

    // FPS calculation
    frame_count++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    if (elapsed >= 1000) {
      fps = frame_count * 1000.0 / elapsed;
      frame_count = 0;
      last_time = now;
    }

    cv::putText(result, "FPS: " + std::to_string(fps).substr(0, 5),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);

    cv::imshow("YOLO v11 Detection", result);
    if (cv::waitKey(1) == 27) {
      break; // ESC key
    }
  }
}

void runTopicMode(std::shared_ptr<rclcpp::Node> node, Yolo11OpenVinoModel& model, 
                  const std::string& input_topic, const std::string& output_topic) {
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic: %s", input_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Publishing to topic: %s", output_topic.c_str());

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(
      output_topic, rclcpp::SensorDataQoS());

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      input_topic, rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV Mat
        cv::Mat frame;
        try {
          frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
          RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }

        if (frame.empty()) {
          RCLCPP_ERROR(node->get_logger(), "Empty frame received from topic %s", input_topic.c_str());
          return;
        }

        // Process frame and get detections
        std::vector<Detection> detections;
        model.setCallback([&detections](const std::vector<Detection>& dets) {
          detections = dets;
        });
        
        model.processImage(frame);
        
        // Draw detections on frame
        cv::Mat result = drawDetections(frame, detections);

        // Publish result
        auto result_msg = cv_bridge::CvImage(msg->header, "bgr8", result).toImageMsg();
        pub->publish(*result_msg);
      });

  RCLCPP_INFO(node->get_logger(), "Vision bounding box node running in topic mode");
  rclcpp::spin(node);
}

auto main(int argc, char **argv) -> int {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vision_boundingbox_node");

  // Declare parameters
  auto webcam = node->declare_parameter("webcam", false);
  auto camera_index = node->declare_parameter("camera_index", 0);
  auto input_topic = node->declare_parameter("input_topic", std::string("/camera/image_raw"));
  auto output_topic = node->declare_parameter("output_topic", std::string("/vision/detections"));
  auto device = node->declare_parameter("device", std::string("CPU"));
  auto height = node->declare_parameter("input_height", 640);
  auto width = node->declare_parameter("input_width", 640);

  RCLCPP_INFO(node->get_logger(), "Vision Bounding Box Node Starting...");
  RCLCPP_INFO(node->get_logger(), "Device: %s", device.c_str());
  RCLCPP_INFO(node->get_logger(), "Input size: %dx%d", width, height);

  try {
    // Create YOLO model
    #if BACKEND == OPENVINO
    Yolo11OpenVinoModel model(node->get_logger(), device, height, width);
    #endif

    if (webcam) {
      RCLCPP_INFO(node->get_logger(), "Running in webcam mode");
      runWebcamMode(node, model, camera_index);
    } else {
      RCLCPP_INFO(node->get_logger(), "Running in topic mode");
      runTopicMode(node, model, input_topic, output_topic);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
