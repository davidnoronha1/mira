#include "app.hpp"
#include <opencv2/imgproc.hpp>

cv::Mat drawDetections(const cv::Mat &frame,
                              const std::vector<Detection> &detections) {
  cv::Mat result = frame.clone();

  for (const auto &det : detections) {
    // Draw bounding box
    cv::rectangle(result, det.box, cv::Scalar(0, 255, 0), 2);

    // Prepare label text
    std::string label =
        det.class_name + ": " + std::to_string(det.confidence).substr(0, 4);

    // Calculate text size and position
    int baseline;
    cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);

    // Draw label background
    cv::Point label_pos(det.box.x, det.box.y - text_size.height - 5);
    cv::rectangle(result,
                  cv::Point(det.box.x, det.box.y - text_size.height - 10),
                  cv::Point(det.box.x + text_size.width, det.box.y),
                  cv::Scalar(0, 255, 0), -1);

    // Draw label text
    cv::putText(result, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 0), 1);
  }

  return result;
}
