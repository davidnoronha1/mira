#include <cv_bridge/cv_bridge.hpp>
#include <future>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#if BACKEND == OPENVINO
#include "backends/openvino/yolo_v11_openvino.hpp"
#endif

using namespace std::chrono_literals;

class VisionBoundingBoxNode : public rclcpp::Node {
public:
  explicit VisionBoundingBoxNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("vision_boundingbox_node", options) {
    RCLCPP_INFO(this->get_logger(), "Vision Bounding Box Node starting up.");
    RCLCPP_INFO(this->get_logger(),
                "This node detects objects from a camera or image topic.");
    RCLCPP_INFO(
        this->get_logger(),
        "Usage: ros2 run vision_boundingbox vision_boundingbox_node --ros-args "
        "-p webcam:=true/false -p visualize:=true/false ...");

    // Declare and get parameters
    webcam_ = this->declare_parameter("webcam", false);
    camera_index_ = this->declare_parameter("camera_index", 0);
    input_topic_ = this->declare_parameter("input_topic",
                                           std::string("/camera/image_raw"));
    device_ = this->declare_parameter("device", std::string("CPU"));
    input_height_ = this->declare_parameter("input_height", 640);
    input_width_ = this->declare_parameter("input_width", 640);
    visualize_ = this->declare_parameter("visualize", false);
    publish_image_ = this->declare_parameter("publish_image", false);

    RCLCPP_INFO(this->get_logger(), "Device: %s", device_.c_str());
    RCLCPP_INFO(this->get_logger(), "Input size: %dx%d", input_width_,
                input_height_);

    try {
#if BACKEND == OPENVINO
      model_ = std::make_unique<Yolo11OpenVinoModel>(
          this->get_logger(), device_, input_height_, input_width_);
#endif
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create YOLO model: %s",
                   e.what());
      throw;
    }

    if (publish_image_) {
      it_pub_ =
          image_transport::create_publisher(this, "/vision/detections/image");
    }

    bbox_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>(
        "/vision/detections/bounding_box", 10);

    // Create start/stop service
    start_stop_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/start_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          if (request->data) {
            this->start();
            response->message = "Detection started.";
          } else {
            this->stop();
            response->message = "Detection stopped.";
          }
          response->success = true;
        });

    start();
  }

  ~VisionBoundingBoxNode() { stop(); }

private:
  void start();
  void stop();

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void timer_callback();
  void process_frame(const cv::Mat &frame);

  // Parameters
  bool webcam_ = false;
  int camera_index_ = 0;
  std::string input_topic_;
  std::string device_;
  int input_height_ = 640;
  int input_width_ = 640;
  bool visualize_ = false;
  bool publish_image_ = false;

  std::unique_ptr<Yolo11OpenVinoModel> model_;

  // Webcam
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS interfaces
  image_transport::Subscriber it_sub_;
  image_transport::Publisher it_pub_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_srv_;

  // Processing queue
  // Create a struct to keep the frame, header, and future tied together
  std::queue<std::future<std::pair<std::vector<Detection>, const cv::Mat>>>
      future_queue_;
  const size_t max_queue_size_ = 3;

  std::atomic<bool> processing_{false};
};

void VisionBoundingBoxNode::start() {
  if (processing_.load()) {
    RCLCPP_INFO(get_logger(), "Detection is already running.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Starting detection...");

  if (webcam_) {
    cap_.open(camera_index_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open camera with index %d",
                   camera_index_);
      return;
    }
    auto image_width = this->get_parameter("image_width").as_int();
    auto image_height = this->get_parameter("image_height").as_int();
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
    timer_ = this->create_wall_timer(
        33ms,
        std::bind(&VisionBoundingBoxNode::timer_callback, this)); // ~30 FPS
  } else {
    it_sub_ = image_transport::create_subscription(
        this, input_topic_,
        std::bind(&VisionBoundingBoxNode::image_callback, this,
                  std::placeholders::_1),
        "raw");
  }

  processing_.store(true);
  RCLCPP_INFO(get_logger(), "Detection started successfully.");
}

void VisionBoundingBoxNode::stop() {
  if (!processing_.load()) {
    RCLCPP_INFO(get_logger(), "Detection is already stopped.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Stopping detection...");
  processing_.store(false);

  if (webcam_) {
    timer_.reset();
    cap_.release();
  } else {
    it_sub_.shutdown();
  }

  // Wait for queue to finish
  // while (!future_queue_.empty()) {
  //   future_queue_.front().wait();
  //   future_queue_.pop();
  // }
  future_queue_ = std::queue<
      std::future<std::pair<std::vector<Detection>, const cv::Mat>>>(); // clear queue

  if (visualize_) {
    cv::destroyAllWindows();
  }

  RCLCPP_INFO(get_logger(), "Detection stopped successfully.");
}

void VisionBoundingBoxNode::timer_callback() {
  if (!processing_.load())
    return;

  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_ERROR(get_logger(), "Failed to read frame from camera");
    return;
  }

  process_frame(frame);
}

void VisionBoundingBoxNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  if (!processing_.load())
    return;

  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  process_frame(frame);
}
void VisionBoundingBoxNode::process_frame(const cv::Mat &frame) {
  // Clean up completed futures
  while (!future_queue_.empty() &&
         future_queue_.front().wait_for(std::chrono::seconds(0)) ==
             std::future_status::ready) {

    auto &task = future_queue_.front();
    auto task_result = task.get();
    auto detections = task_result.first;
    auto original_frame = task_result.second;

    vision_msgs::msg::BoundingBox2DArray bbox_array_msg;
    bbox_array_msg.header =
        std_msgs::msg::Header(); // You can set this to the appropriate header
                                 // if needed
    bbox_array_msg.header.frame_id =
        "camera"; // Set frame_id if you have one in the header

    for (const auto &det : detections) {
      vision_msgs::msg::BoundingBox2D bbox;
      bbox.center.position.x = det.box.x + det.box.width / 2.0;
      bbox.center.position.y = det.box.y + det.box.height / 2.0;
      bbox.size_x = det.box.width;
      bbox.size_y = det.box.height;
      bbox_array_msg.boxes.push_back(bbox);
    }
    bbox_pub_->publish(bbox_array_msg);

    if (publish_image_ || visualize_) {
      // Draw on the stored frame, not the newest incoming frame
      cv::Mat result_img = drawDetections(task_result.second, detections);

      if (publish_image_) {
        auto result_msg =
            cv_bridge::CvImage(bbox_array_msg.header, "bgr8", result_img).toImageMsg();
        it_pub_.publish(*result_msg); // Dereference may be needed depending on
                                      // cv_bridge version
      }
      if (visualize_) {
        cv::imshow("YOLO Detections", result_img);
        cv::waitKey(1);
      }
    }
    future_queue_.pop();
  }

  // Only push new frames if queue isn't too full
  if (future_queue_.size() < max_queue_size_) {
    auto future = model_->processImage(frame);
    // Use frame.clone() to ensure OpenCV memory isn't overwritten by the next
    // camera read
    future_queue_.push(std::move(future));
  }
}

auto main(int argc, char **argv) -> int {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<VisionBoundingBoxNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
