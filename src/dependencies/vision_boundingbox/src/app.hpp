#pragma once
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <vector>

struct Detection {
  cv::Rect box;
  float confidence;
  int class_id;
  std::string class_name;
};

class ImageProcessor {
protected:
  rclcpp::Logger logger_;
  std::function<void(const std::vector<Detection>&)> callback_;
  int input_height_;
  int input_width_;
  std::string device_;

public:
  ImageProcessor(rclcpp::Logger logger, const std::string &device = "CPU", 
             int height = 640, int width = 640,
             std::function<void(const std::vector<Detection>&)> callback = nullptr)
      : logger_(logger), callback_(callback), 
        input_height_(height), input_width_(width), device_(device) {
  }

  virtual ~ImageProcessor() = default;

  virtual void processImage(const cv::Mat &image) = 0;
  
  void setCallback(std::function<void(const std::vector<Detection>&)> callback) {
    callback_ = callback;
  }
};

cv::Mat drawDetections(const cv::Mat &frame,
                              const std::vector<Detection> &detections);