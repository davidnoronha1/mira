#include "yolo_v11_openvino.hpp"
#include <functional>
#include <utility>
#include <vector>
#include <fstream>



Yolo11OpenVinoModel::Yolo11OpenVinoModel(rclcpp::Logger logger, const std::string &device, 
            int height, int width,
            std::function<void(const std::vector<Detection>&)> callback)
    : ImageProcessor(logger, device, height, width, std::move(callback)), 
      input_height_(height), input_width_(width) {
  package_share_dir = ament_index_cpp::get_package_share_directory("vision_boundingbox");
  std::string model_path;

    RCLCPP_INFO(logger_, "Initializing YOLOv11 on device: %s", device_.c_str());
    RCLCPP_INFO(logger_, "Input image height: %d, width: %d", input_height_, input_width_);

    model_path = package_share_dir + "/models/yolo11n_openvino_model_int8/yolo11n.xml";
    RCLCPP_INFO(logger_, "Loading INT8 model from %s", model_path.c_str());

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
      if (device_ == "CPU" || device_ == "GPU") {
        RCLCPP_INFO(logger_, "Using OpenVINO preprocessing");

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
        RCLCPP_INFO(logger_, "Loaded model with OpenVINO preprocessing");
      } else {
        RCLCPP_INFO(logger_, "Using basic preprocessing");
        input_port_ = raw_compiled_model_.input();
        output_port_ = raw_compiled_model_.output();
        use_ov_preprocess_ = false;
      }

      // Print model info
      RCLCPP_INFO(logger_, "Model loaded successfully on %s", device_.c_str());

    } catch (const std::exception &e) {
      throw std::runtime_error(
          std::string("Failed to load YOLO model: ") + e.what());
    }
  }

 cv::Mat Yolo11OpenVinoModel::cpuPreprocess(const cv::Mat &frame) {
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

void Yolo11OpenVinoModel::processImage(const cv::Mat &image) {
  auto req = processFrameAsync(image, [this](std::vector<Detection> detections) {
    if (callback_) {
      callback_(detections);
    }
  });
  req.infer();
}

void Yolo11OpenVinoModel::initializeClassNames() {
    std::string class_names_path = package_share_dir + "/models/class_names.txt";

    std::ifstream file(class_names_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open class names file: %s", class_names_path.c_str());
      RCLCPP_INFO(logger_, "Using default COCO class names");
      
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
    
    RCLCPP_INFO(logger_, "Loaded %zu class names from %s", class_names_.size(), class_names_path.c_str());
  }

std::vector<Detection> Yolo11OpenVinoModel::postProcess(float *output_data, const ov::Shape &output_shape,
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
      float *detection = output_data + i * (num_classes + 5);
      
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

ov::InferRequest Yolo11OpenVinoModel::createInferRequest(const cv::Mat &frame) {
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

ov::InferRequest Yolo11OpenVinoModel::processFrameAsync(const cv::Mat &frame,
                                   std::function<void(std::vector<Detection>)> callback) {
    float scale_x = static_cast<float>(frame.cols) / 640.0f;
    float scale_y = static_cast<float>(frame.rows) / 640.0f;

    auto req = this->createInferRequest(frame);
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
      callback(detections);
    });
    req.start_async();
    return req;
  }
