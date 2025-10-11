#include "app.hpp"
#include <openvino/openvino.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vision_boundingbox/clog.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

struct Detection {
  cv::Rect box;
  float confidence;
  int class_id;
  std::string class_name;
};

class YOLO11 {
public:
  YOLO11(ov::Core &core, const std::string &device, int height, int width)
      : device_(device), input_height_(height), input_width_(width) {
    std::string package_share_dir =
        ament_index_cpp::get_package_share_directory("vision_boundingbox");
    std::string model_path;

    plog::info() << "Initializing YOLOv11 on device: " << device;
    plog::info() << "Input image height: " << height << ", width: " << width;

    model_path = package_share_dir + "/models/yolo11n_openvino_model_int8/yolo11n.xml";
    plog::info() << "Loading INT8 model from " << model_path;

    // Initialize class names for COCO dataset
    initializeClassNames();

    try {
      // Read model
      std::shared_ptr<ov::Model> model = core.read_model(model_path);

      // Reshape model for YOLO input
      model->reshape(ov::PartialShape{1, 3, 640, 640});

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

        // YOLO preprocessing steps
        ppp.input()
            .preprocess()
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR, 640, 640)
            .convert_element_type(ov::element::f32)
            .scale({255.0f});

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
          std::string("Failed to load YOLO model: ") + e.what());
    }
  }

  static auto cpuPreprocess(const cv::Mat &frame) -> cv::Mat {
    // Resize to 640x640
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(640, 640), 0, 0, cv::INTER_LINEAR);

    // Convert BGR -> RGB
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Normalize to [0, 1]
    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    // HWC -> CHW
    std::vector<cv::Mat> chw(3);
    cv::split(resized, chw);

    cv::Mat blob(1, 3 * 640 * 640, CV_32F);
    float *blob_data = blob.ptr<float>();
    for (int i = 0; i < 3; i++) {
      std::memcpy(blob_data + i * 640 * 640, chw[i].data,
                  640 * 640 * sizeof(float));
    }

    return blob;
  }

private:
  void initializeClassNames() {
    std::string package_share_dir =
        ament_index_cpp::get_package_share_directory("vision_boundingbox");
    std::string class_names_path = package_share_dir + "/models/class_names.txt";
    
    std::ifstream file;
	file.open(class_names_path);
    if (true) {
      plog::error() << "Failed to open class names file: " << class_names_path;
      plog::info() << "Using default COCO class names";
      
      // Fallback to default COCO classes
      class_names_ = {
          "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
          "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
          "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
          "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
          "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
          "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
          "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
          "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
          "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
          "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
          "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
          "toothbrush"
      };
      return;
    }
    
    class_names_.clear();
    std::string line;
    while (std::getline(file, line)) {
      // Remove trailing whitespace and carriage return
      line.erase(line.find_last_not_of(" \t\r\n") + 1);
      if (!line.empty()) {
        class_names_.push_back(line);
      }
    }
    file.close();
    
    plog::info() << "Loaded " << class_names_.size() << " class names from " << class_names_path;
  }

  std::vector<Detection> postProcess(const float *output_data, const ov::Shape &output_shape,
                                   float conf_threshold, float nms_threshold,
                                   float scale_x, float scale_y) {
    std::vector<Detection> detections;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    // Parse YOLO output (assuming format: [batch, classes+5, num_detections])
    int num_classes = class_names_.size();
    int num_detections = output_shape[2];

    for (int i = 0; i < num_detections; i++) {
      float *detection = (float*)output_data + i * (num_classes + 5);
      
      float x = detection[0];
      float y = detection[1];
      float w = detection[2];
      float h = detection[3];
      float confidence = detection[4];

      if (confidence > conf_threshold) {
        // Find best class
        float max_class_score = 0;
        int best_class_id = 0;
        for (int j = 0; j < num_classes; j++) {
          if (detection[5 + j] > max_class_score) {
            max_class_score = detection[5 + j];
            best_class_id = j;
          }
        }

        float final_confidence = confidence * max_class_score;
        if (final_confidence > conf_threshold) {
          // Convert from center coordinates to top-left
          int left = static_cast<int>((x - w / 2) * scale_x);
          int top = static_cast<int>((y - h / 2) * scale_y);
          int width = static_cast<int>(w * scale_x);
          int height = static_cast<int>(h * scale_y);

          boxes.push_back(cv::Rect(left, top, width, height));
          confidences.push_back(final_confidence);
          class_ids.push_back(best_class_id);
        }
      }
    }

    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    for (int idx : indices) {
      Detection det;
      det.box = boxes[idx];
      det.confidence = confidences[idx];
      det.class_id = class_ids[idx];
      det.class_name = (class_ids[idx] < class_names_.size()) ? 
                       class_names_[class_ids[idx]] : "unknown";
      detections.push_back(det);
    }

    return detections;
  }

  static cv::Mat drawDetections(const cv::Mat &frame, const std::vector<Detection> &detections) {
    cv::Mat result = frame.clone();
    
    for (const auto &det : detections) {
      // Draw bounding box
      cv::rectangle(result, det.box, cv::Scalar(0, 255, 0), 2);
      
      // Prepare label text
      std::string label = det.class_name + ": " + 
                         std::to_string(det.confidence).substr(0, 4);
      
      // Calculate text size and position
      int baseline;
      cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
      
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
    float scale_x = static_cast<float>(frame.cols) / 640.0f;
    float scale_y = static_cast<float>(frame.rows) / 640.0f;

    ov::Tensor output_tensor;
    auto req = createInferRequest(frame);
    req.infer();
    output_tensor = req.get_output_tensor();

    float *output_data = output_tensor.data<float>();
    ov::Shape output_shape = output_tensor.get_shape();
    
    auto detections = postProcess(output_data, output_shape, 0.5f, 0.4f, scale_x, scale_y);
    return drawDetections(frame, detections);
  }

public:
  ov::InferRequest processFrameAsync(const cv::Mat &frame,
                                     std::function<void(cv::Mat)> callback) {
    float scale_x = static_cast<float>(frame.cols) / 640.0f;
    float scale_y = static_cast<float>(frame.rows) / 640.0f;
    
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
      float *output_data = output_tensor.data<float>();
      ov::Shape output_shape = output_tensor.get_shape();
      
      auto detections = postProcess(output_data, output_shape, 0.5f, 0.4f, scale_x, scale_y);
      cv::Mat result = drawDetections(frame, detections);
      callback(result);
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
    float scale_x = 1.0f, scale_y = 1.0f;
    bool has_previous = false;

    while (true) {
      cv::Mat frame;
      if (!cap.read(frame)) {
        plog::error() << "Failed to read frame from camera";
        break;
      }

      // Calculate scaling factors
      float new_scale_x = static_cast<float>(frame.cols) / 640.0f;
      float new_scale_y = static_cast<float>(frame.rows) / 640.0f;

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
        float *output_data = output_tensor.data<float>();
        ov::Shape output_shape = output_tensor.get_shape();
        
        auto detections = postProcess(output_data, output_shape, 0.5f, 0.4f, scale_x, scale_y);
        cv::Mat result = drawDetections(current_frame, detections);

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

        cv::putText(result, "FPS: " + std::to_string(fps).substr(0, 5),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 0), 2);

        cv::imshow("YOLO v11 Detection", result);
        if (cv::waitKey(1) == 27)
          break; // ESC key
      }

      // Update for next iteration
      current_infer_request = new_infer_request;
      current_frame = frame.clone();
      scale_x = new_scale_x;
      scale_y = new_scale_y;
      has_previous = true;
    }

    // Handle last frame
    if (has_previous) {
      current_infer_request.wait();
      ov::Tensor output_tensor = current_infer_request.get_output_tensor();
      float *output_data = output_tensor.data<float>();
      ov::Shape output_shape = output_tensor.get_shape();
      
      auto detections = postProcess(output_data, output_shape, 0.5f, 0.4f, scale_x, scale_y);
      cv::Mat result = drawDetections(current_frame, detections);
      cv::imshow("YOLO v11 Detection", result);
      cv::waitKey(1);
    }
  }

private:
  std::string device_;
  int input_height_;
  int input_width_;
  bool use_ov_preprocess_;
  ov::CompiledModel compiled_model_;
  ov::CompiledModel raw_compiled_model_;
  ov::Output<const ov::Node> input_port_;
  ov::Output<const ov::Node> output_port_;
  std::vector<std::string> class_names_;
};

void cv_camera_yolov11(std::shared_ptr<rclcpp::Node> nh, ov::Core &core) {
	std::string device = nh->declare_parameter<std::string>("device", "CPU");
	int camera_index = nh->declare_parameter<int>("camera_index", 0);
	auto image_width = nh->declare_parameter<int>("image_width", 640);
	auto image_height = nh->declare_parameter<int>("image_height", 480);

	YOLO11 yolov11(core, device, image_height, image_width);

	cv::VideoCapture cap(camera_index);
	if (!cap.isOpened()) {
		plog::error() << "Failed to open camera with index " << camera_index;
		return;
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

	plog::info() << "Starting camera capture on index " << camera_index;

	yolov11.asyncRun(cap);
}

void topic_camera_yolov11(std::shared_ptr<rclcpp::Node> nh, ov::Core &core,
						  const std::string &topic) {
  std::string device = nh->declare_parameter<std::string>("device", "CPU");
  auto image_width = nh->declare_parameter<int>("image_width", 640);
  auto image_height = nh->declare_parameter<int>("image_height", 480);

  YOLO11 yolov11(core, device, image_height, image_width);

  auto pub = nh->create_publisher<sensor_msgs::msg::Image>(
	  "/ml_boundingbox", rclcpp::SensorDataQoS());
  auto sub = nh->create_subscription<sensor_msgs::msg::Image>(
	  topic, rclcpp::SensorDataQoS(),
	  [&](const sensor_msgs::msg::Image::SharedPtr msg) {
		// Convert ROS image to OpenCV Mat
		cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		if (frame.empty()) {
		  plog::error() << "Empty frame received from topic " << topic;
		  return;
		}

		yolov11.processFrameAsync(frame, [&](const cv::Mat& result_vis) {
		  auto result_msg = cv_bridge::CvImage(
			  std_msgs::msg::Header(), "bgr8", result_vis)
			  .toImageMsg();
		  pub->publish(*result_msg);
		});
	  });
}