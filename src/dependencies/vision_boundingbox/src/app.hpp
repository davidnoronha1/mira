#pragma once
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <future>

struct Detection {
  cv::Rect box;
  float confidence;
  int class_id;
  std::string class_name;
};

class ImageProcessor {
protected:
  rclcpp::Logger logger_;
  int input_height_;
  int input_width_;
  std::string device_;

public:
  ImageProcessor(rclcpp::Logger logger, const std::string &device = "CPU", 
             int height = 640, int width = 640)
      : logger_(logger), 
        input_height_(height), input_width_(width), device_(device) {
  }

  virtual ~ImageProcessor() = default;

  virtual std::future<std::vector<Detection>> processImage(const cv::Mat &image) = 0;

 
};

class YOLOImageProcessor : public ImageProcessor {
   void initializeClassNames();
};

cv::Mat drawDetections(const cv::Mat &frame,
                              const std::vector<Detection> &detections);