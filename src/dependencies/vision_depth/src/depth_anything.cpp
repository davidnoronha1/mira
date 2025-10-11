#include "app.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_depth/clog.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <exception>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/infer_request.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

class YOLO11 {
public:
  YOLO11(ov::Core &core, const std::string &device, int height,
                int width)
      : device_(device) {
    std::string package_share_dir =
        ament_index_cpp::get_package_share_directory("vision_depth");
    std::string model_path;

    plog::info() << "Initializing DepthAnything on device: " << device;
    plog::info() << "Input image height: " << height << ", width: " << width;

    model_path =
        package_share_dir + "/models/depth-anything/depth-anything_int8.xml";
    plog::info() << "Loading INT8 model from " << model_path;

    try {
      // Read model
      std::shared_ptr<ov::Model> model = core.read_model(model_path);

      // Reshape model
      model->reshape(ov::PartialShape{1, 3, 518, 518});

      // Store raw compiled model for NPU
      raw_compiled_model_ = core.compile_model(model, device);

      // Use OpenVINO preprocessing for CPU and GPU
      if (device == "CPU" || device == "GPU") {
        plog::info() << "Using OpenVINO preprocessing";

        ov::preprocess::PrePostProcessor ppp(model);

        // Input tensor configuration
        ppp.input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_shape({1, height, width, 3})
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);

        // Model expects NCHW
        ppp.input().model().set_layout("NCHW");

        // Preprocessing steps
        ppp.input()
            .preprocess()
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR, 518, 518)
            .convert_element_type(ov::element::f32)
            .mean({0.485f * 255, 0.456f * 255, 0.406f * 255})
            .scale({0.229f * 255, 0.224f * 255, 0.225f * 255});

        auto final_model = ppp.build();
        compiled_model_ = core.compile_model(final_model, device);
        input_port_ = compiled_model_.input();
        output_port_ = compiled_model_.output();

        use_ov_preprocess_ = true;
        plog::info() << "Loaded model with OpenVINO preprocessing";
      } else {
        plog::info() << "Using basic preprocessing";
        input_port_ = raw_compiled_model_.input();
        output_port_ = raw_compiled_model_.output();
        use_ov_preprocess_ = false;
      }

      // Print model info
      plog::info() << "Model loaded successfully on " << device;

    } catch (const std::exception &e) {
      throw std::runtime_error(
          std::string("Failed to load DepthAnything model: ") + e.what());
    }
  }

  auto cpuPreprocess(const cv::Mat &frame) -> cv::Mat {
    // Resize to 518x518
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(518, 518), 0, 0, cv::INTER_CUBIC);

    // Convert BGR -> RGB
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Normalize
    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    std::vector<float> mean = {0.485f, 0.456f, 0.406f};
    std::vector<float> std = {0.229f, 0.224f, 0.225f};

    for (int y = 0; y < resized.rows; y++) {
      cv::Vec3f *row = resized.ptr<cv::Vec3f>(y);
      for (int x = 0; x < resized.cols; x++) {
        row[x][0] = (row[x][0] - mean[0]) / std[0];
        row[x][1] = (row[x][1] - mean[1]) / std[1];
        row[x][2] = (row[x][2] - mean[2]) / std[2];
      }
    }

    // HWC -> CHW
    std::vector<cv::Mat> chw(3);
    cv::split(resized, chw);

    cv::Mat blob(1, 3 * 518 * 518, CV_32F);
    float *blob_data = blob.ptr<float>();
    for (int i = 0; i < 3; i++) {
      std::memcpy(blob_data + i * 518 * 518, chw[i].data,
                  518 * 518 * sizeof(float));
    }

    return blob;
  }

private:
  static cv::Mat getDepthMap(const float *data, int depth_h, int depth_w, int orig_w,
                      int orig_h) {
    // Create depth mat from output
    cv::Mat depth(depth_h, depth_w, CV_32F, (void *)data);
    // std::memcpy(depth.data, data, depth_h * depth_w * sizeof(float));

    // Resize to original dimensions
    // cv::resize(depth, depth, cv::Size(orig_w, orig_h), 0, 0,
    // cv::INTER_LINEAR);

    // Normalize with better handling
    double depth_min = 0;
    double depth_max = 0;
    cv::minMaxLoc(depth, &depth_min, &depth_max);

    // cv::Mat depth_normalized;
    depth = (depth - depth_min) / (depth_max - depth_min);

    // Convert to uint8
    cv::Mat depth_uint8;
    depth.convertTo(depth_uint8, CV_8U, 255.0);

    // Apply colormap (INFERNO)
    cv::Mat depth_color;
    cv::applyColorMap(depth_uint8, depth_color, cv::COLORMAP_INFERNO);

    return depth_color;
  }

  ov::InferRequest createInferRequest(const cv::Mat &frame) {
    ov::InferRequest infer_request;
    if (use_ov_preprocess_) {
      infer_request = compiled_model_.create_infer_request();
      ov::Tensor input_tensor(ov::element::u8,
                              {1, static_cast<size_t>(frame.rows),
                               static_cast<size_t>(frame.cols), 3},
                              frame.data);
      infer_request.set_input_tensor(input_tensor);
    } else {
      infer_request = raw_compiled_model_.create_infer_request();
      cv::Mat input_blob = cpuPreprocess(frame);
      ov::Tensor input_tensor(input_port_.get_element_type(),
                              input_port_.get_shape(), input_blob.ptr<float>());
      infer_request.set_input_tensor(input_tensor);
    }
    return infer_request;
  }

  cv::Mat processFrameSync(const cv::Mat &frame) {
    int orig_w = frame.cols;
    int orig_h = frame.rows;

    ov::Tensor output_tensor;
    auto req = createInferRequest(frame);
    req.infer();
    output_tensor = req.get_output_tensor();

    float *depth_values = output_tensor.data<float>();
    // std::cout << "Output shape: " << output_tensor.get_shape() << std::endl;
    return getDepthMap(depth_values, 518, 686, orig_w, orig_h);
  }

public:
  ov::InferRequest processFrameAsync(const cv::Mat &frame,
                                     std::function<void(cv::Mat)> callback) {
    auto req = createInferRequest(frame);
    req.set_callback([&](std::exception_ptr exception_ptr) {
      if (exception_ptr) {
        try {
          std::rethrow_exception(exception_ptr);
        } catch (const std::exception &e) {
          std::cerr << "Inference error: " << e.what() << std::endl;
        }
        return;
      }

      ov::Tensor output_tensor = req.get_output_tensor();
      float *depth_values = output_tensor.data<float>();
      cv::Mat depth_vis =
          getDepthMap(depth_values, 518, 686, frame.cols, frame.rows);
      callback(depth_vis);
    });
    req.start_async();
    return req;
  }

  void asyncRun(cv::VideoCapture &cap) {
    int frame_count = 0;
    double fps = 0.0;
    auto last_time = std::chrono::steady_clock::now();

    ov::InferRequest current_infer_request;
    cv::Mat current_frame;
    int current_w = 0, current_h = 0;
    bool has_previous = false;

    while (true) {
      cv::Mat frame;
      if (!cap.read(frame)) {
        plog::error() << "Failed to read frame from camera";
        break;
      }

      // Start async inference for current frame
      ov::InferRequest new_infer_request;
      if (use_ov_preprocess_) {
        new_infer_request = compiled_model_.create_infer_request();
        ov::Tensor input_tensor(ov::element::u8,
                                {1, static_cast<size_t>(frame.rows),
                                 static_cast<size_t>(frame.cols), 3},
                                frame.data);
        new_infer_request.set_input_tensor(input_tensor);
      } else {
        new_infer_request = raw_compiled_model_.create_infer_request();
        cv::Mat input_blob = cpuPreprocess(frame);
        ov::Tensor input_tensor(input_port_.get_element_type(),
                                input_port_.get_shape(),
                                input_blob.ptr<float>());
        new_infer_request.set_input_tensor(input_tensor);
      }
      new_infer_request.start_async();

      // If we have a previous inference running, wait for it and display result
      if (has_previous) {
        current_infer_request.wait();

        ov::Tensor output_tensor = current_infer_request.get_output_tensor();

        cv::Mat depth_vis = getDepthMap(output_tensor.data<float>(), 518, 686,
                                        current_w, current_h);

        // FPS calculation
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_time)
                           .count();
        if (elapsed >= 1000) {
          fps = frame_count * 1000.0 / elapsed;
          frame_count = 0;
          last_time = now;
        }

        cv::putText(depth_vis, "FPS: " + std::to_string(fps).substr(0, 5),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 0), 2);

        cv::imshow("Depth Anything V2", depth_vis);
        if (cv::waitKey(1) == 27)
          break; // ESC key
      }

      // Update for next iteration
      current_infer_request = new_infer_request;
      current_w = frame.cols;
      current_h = frame.rows;
      has_previous = true;
    }

    // Handle last frame
    if (has_previous) {
      current_infer_request.wait();
      ov::Tensor output_tensor = current_infer_request.get_output_tensor();
      ov::Shape out_shape = output_tensor.get_shape();

      cv::Mat depth_vis = getDepthMap(output_tensor.data<float>(), out_shape[2],
                                      out_shape[3], current_w, current_h);
      cv::imshow("Depth Anything V2", depth_vis);
      cv::waitKey(1);
    }
  }

private:
  std::string device_;
  bool use_ov_preprocess_;
  ov::CompiledModel compiled_model_;
  ov::CompiledModel raw_compiled_model_;
  ov::Output<const ov::Node> input_port_;
  ov::Output<const ov::Node> output_port_;
};

void cv_camera_depth(std::shared_ptr<rclcpp::Node> nh, ov::Core &core) {
  std::string device = nh->declare_parameter<std::string>("device", "CPU");
  int camera_index = nh->declare_parameter<int>("camera_index", 0);
  auto image_width = nh->declare_parameter<int>("image_width", 640);
  auto image_height = nh->declare_parameter<int>("image_height", 480);

  YOLO11 depth_anything(core, device, image_height, image_width);

  cv::VideoCapture cap(camera_index);
  if (!cap.isOpened()) {
    plog::error() << "Failed to open camera with index " << camera_index;
    return;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

  plog::info() << "Starting camera capture on index " << camera_index;

  depth_anything.asyncRun(cap);
}

void topic_camera_depth(std::shared_ptr<rclcpp::Node> nh, ov::Core &core,
                        const std::string &topic) {
  std::string device = nh->declare_parameter<std::string>("device", "CPU");
  auto image_width = nh->declare_parameter<int>("image_width", 640);
  auto image_height = nh->declare_parameter<int>("image_height", 480);
  
  YOLO11 depth_anything(core, device, image_height, image_width);

  auto pub = nh->create_publisher<sensor_msgs::msg::Image>(
      "/ml_depth", rclcpp::SensorDataQoS());
  auto sub = nh->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV Mat
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (frame.empty()) {
          plog::error() << "Empty frame received from topic " << topic;
          return;
        }

        depth_anything.processFrameAsync(frame, [&](const cv::Mat& depth_vis) {
          auto depth_msg = cv_bridge::CvImage(
              std_msgs::msg::Header(), "32FC1", depth_vis)
              .toImageMsg();
          pub->publish(*depth_msg);
        });
      });
}