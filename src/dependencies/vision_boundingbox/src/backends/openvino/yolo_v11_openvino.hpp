#pragma once

#include "../../app.hpp"
#include <functional>
#include <openvino/openvino.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// Forward declaration
class Yolo11OpenVinoModel : public ImageProcessor {
  ov::Core core;
  std::string package_share_dir;
  int input_height_;
  int input_width_;
  bool use_ov_preprocess_;
  ov::CompiledModel compiled_model_;
  ov::CompiledModel raw_compiled_model_;
  ov::Output<const ov::Node> input_port_;
  ov::Output<const ov::Node> output_port_;
  std::vector<std::string> class_names_;

public:
  Yolo11OpenVinoModel(rclcpp::Logger logger, const std::string &device = "CPU", 
              int height = 640, int width = 640,
              std::function<void(const std::vector<Detection>&)> callback = nullptr);

  void processImage(const cv::Mat &image) override;

  ov::InferRequest processFrameAsync(const cv::Mat &frame,
                                     std::function<void(std::vector<Detection>)> callback);

private:
  void initializeClassNames();
  
  static cv::Mat cpuPreprocess(const cv::Mat &frame);
  
  std::vector<Detection> postProcess(float *output_data, const ov::Shape &output_shape,
                                     float conf_threshold, float nms_threshold,
                                     float scale_x, float scale_y);
  
  ov::InferRequest createInferRequest(const cv::Mat &frame);
};
