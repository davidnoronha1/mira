#include "yolo_v11_tensorrt.hpp"

#include <fstream>
#include <stdexcept>
#include <cstring>

// ─── CUDA error-check helper ──────────────────────────────────────────────────

#define CUDA_CHECK(call)                                                        \
    do {                                                                        \
        cudaError_t _err = (call);                                              \
        if (_err != cudaSuccess) {                                              \
            throw std::runtime_error(                                           \
                std::string("CUDA error at " __FILE__ ":")                     \
                + std::to_string(__LINE__) + " — "                             \
                + cudaGetErrorString(_err));                                    \
        }                                                                       \
    } while (0)

// ─── TRTLogger ────────────────────────────────────────────────────────────────

void TRTLogger::log(Severity severity, const char* msg) noexcept
{
    switch (severity) {
        case Severity::kINTERNAL_ERROR:
        case Severity::kERROR:
            RCLCPP_ERROR(ros_logger_, "[TRT] %s", msg); break;
        case Severity::kWARNING:
            RCLCPP_WARN (ros_logger_, "[TRT] %s", msg); break;
        case Severity::kINFO:
            RCLCPP_INFO (ros_logger_, "[TRT] %s", msg); break;
        default: break;  // kVERBOSE — suppress
    }
}

// ─── Constructor ──────────────────────────────────────────────────────────────

Yolo11TensorRTModel::Yolo11TensorRTModel(rclcpp::Logger     logger,
                                         const std::string& device,
                                         int                height,
                                         int                width)
    : ImageProcessor(logger, device, height, width)
{
    package_share_dir_ =
        ament_index_cpp::get_package_share_directory("vision_boundingbox");

    RCLCPP_INFO(logger_, "Initialising YOLOv11 TensorRT backend  (%dx%d)",
                height, width);

    initializeClassNames();

    trt_logger_ = std::make_unique<TRTLogger>(logger_);

    const std::string engine_path =
        package_share_dir_ + "/models/yolo11n_trt/yolo11n.engine";

    loadEngine(engine_path);
    resolveBindings();

    RCLCPP_INFO(logger_, "TensorRT model ready — "
                "input %zu bytes, output %zu bytes  "
                "(%d anchors, %d values/anchor)",
                input_size_bytes_, output_size_bytes_,
                output_num_detections_, output_num_values_);
}

// ─── Engine load ──────────────────────────────────────────────────────────────

void Yolo11TensorRTModel::loadEngine(const std::string& engine_path)
{
    RCLCPP_INFO(logger_, "Loading TRT engine: %s", engine_path.c_str());

    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file.good())
        throw std::runtime_error("Engine file not found: " + engine_path);

    const std::streamsize fsize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> data(fsize);
    if (!file.read(data.data(), fsize))
        throw std::runtime_error("Failed to read engine file: " + engine_path);

    runtime_.reset(nvinfer1::createInferRuntime(*trt_logger_));
    if (!runtime_)
        throw std::runtime_error("createInferRuntime failed");

    engine_.reset(runtime_->deserializeCudaEngine(data.data(), fsize, nullptr));
    if (!engine_)
        throw std::runtime_error("deserializeCudaEngine failed for: " + engine_path);

    context_.reset(engine_->createExecutionContext());
    if (!context_)
        throw std::runtime_error("createExecutionContext failed");

    RCLCPP_INFO(logger_, "Engine loaded successfully");
}

// ─── Binding resolution ───────────────────────────────────────────────────────

void Yolo11TensorRTModel::resolveBindings()
{
    const int nb = engine_->getNbBindings();
    for (int i = 0; i < nb; ++i) {
        if (engine_->bindingIsInput(i))
            input_binding_idx_ = i;
        else
            output_binding_idx_ = i;
    }

    if (input_binding_idx_ < 0 || output_binding_idx_ < 0)
        throw std::runtime_error("Could not locate input/output bindings in engine");

    // Input: [1, 3, H, W]  float32
    {
        auto dims = engine_->getBindingDimensions(input_binding_idx_);
        input_size_bytes_ = sizeof(float);
        for (int d = 0; d < dims.nbDims; ++d)
            input_size_bytes_ *= static_cast<size_t>(dims.d[d]);
    }

    // Output: [1, 4+classes, num_anchors]  float32
    {
        auto dims = engine_->getBindingDimensions(output_binding_idx_);
        output_num_values_     = dims.d[1];   // 4 + num_classes
        output_num_detections_ = dims.d[2];   // e.g. 8400

        output_size_bytes_ = sizeof(float);
        for (int d = 0; d < dims.nbDims; ++d)
            output_size_bytes_ *= static_cast<size_t>(dims.d[d]);
    }
}

// ─── CPU preprocessing ────────────────────────────────────────────────────────

std::vector<float> Yolo11TensorRTModel::cpuPreprocess(const cv::Mat& frame) const
{
    cv::Mat resized;
    cv::resize(frame, resized,
               cv::Size(MODEL_INPUT_W, MODEL_INPUT_H),
               0, 0, cv::INTER_LINEAR);

    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    // HWC → CHW
    std::vector<cv::Mat> channels(3);
    cv::split(resized, channels);

    std::vector<float> blob(3 * MODEL_INPUT_H * MODEL_INPUT_W);
    float* ptr = blob.data();
    for (int c = 0; c < 3; ++c) {
        std::memcpy(ptr, channels[c].data,
                    MODEL_INPUT_H * MODEL_INPUT_W * sizeof(float));
        ptr += MODEL_INPUT_H * MODEL_INPUT_W;
    }
    return blob;
}

// ─── Post-processing ──────────────────────────────────────────────────────────

std::vector<Detection> Yolo11TensorRTModel::postProcess(
    const float* output_data,
    int          num_detections,
    int          num_classes_plus4,
    float        conf_threshold,
    float        nms_threshold,
    float        scale_x,
    float        scale_y) const
{
    const int num_classes = num_classes_plus4 - 4;

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      class_ids;

    boxes.reserve(num_detections);
    confidences.reserve(num_detections);
    class_ids.reserve(num_detections);

    for (int i = 0; i < num_detections; ++i) {
        // Layout: [num_values × num_detections] — column-major per anchor
        const float cx = output_data[0 * num_detections + i];
        const float cy = output_data[1 * num_detections + i];
        const float bw = output_data[2 * num_detections + i];
        const float bh = output_data[3 * num_detections + i];

        float best_score = 0.0f;
        int   best_class = 0;
        for (int c = 0; c < num_classes; ++c) {
            const float score = output_data[(4 + c) * num_detections + i];
            if (score > best_score) {
                best_score = score;
                best_class = c;
            }
        }

        if (best_score < conf_threshold) continue;

        const int left = static_cast<int>((cx - bw / 2.0f) * scale_x);
        const int top  = static_cast<int>((cy - bh / 2.0f) * scale_y);
        const int iw   = static_cast<int>(bw * scale_x);
        const int ih   = static_cast<int>(bh * scale_y);

        boxes.emplace_back(left, top, iw, ih);
        confidences.push_back(best_score);
        class_ids.push_back(best_class);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    std::vector<Detection> detections;
    detections.reserve(indices.size());
    for (int idx : indices) {
        Detection det;
        det.box        = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id   = class_ids[idx];
        det.class_name = (class_ids[idx] < static_cast<int>(class_names_.size()))
                             ? class_names_[class_ids[idx]]
                             : "unknown";
        detections.push_back(std::move(det));
    }
    return detections;
}

// ─── CUDA stream callback ─────────────────────────────────────────────────────
// Called by the CUDA driver thread once all stream work has finished.
// Must be static — no implicit `this`. Keep it short; heavy work blocks the driver.

void CUDART_CB Yolo11TensorRTModel::onInferenceComplete(
    cudaStream_t  /*stream*/,
    cudaError_t   status,
    void*         userdata)
{
    // Re-own the CallbackData — will be destroyed when this scope exits,
    // which in turn destroys InferenceContext and frees all CUDA resources.
    std::unique_ptr<CallbackData> cbd(static_cast<CallbackData*>(userdata));

    if (status != cudaSuccess) {
        cbd->ctx->promise.set_exception(
            std::make_exception_ptr(std::runtime_error(
                std::string("CUDA stream error: ") + cudaGetErrorString(status))));
        return;
    }

    try {
        auto detections = cbd->self->postProcess(
            cbd->ctx->h_output.data(),
            cbd->self->output_num_detections_,
            cbd->self->output_num_values_,
            CONF_THRESHOLD, NMS_THRESHOLD,
            cbd->scale_x, cbd->scale_y);

        cbd->ctx->promise.set_value(std::make_pair(std::move(detections), cbd->image));
    } catch (...) {
        cbd->ctx->promise.set_exception(std::current_exception());
    }
}

// ─── processImage ─────────────────────────────────────────────────────────────

std::future<std::vector<Detection>>
Yolo11TensorRTModel::processImage(const cv::Mat& image)
{
    // Scale factors: map 640×640 coords back to original resolution
    const float scale_x = static_cast<float>(image.cols) / MODEL_INPUT_W;
    const float scale_y = static_cast<float>(image.rows) / MODEL_INPUT_H;

    // ── Preprocess on the calling thread (pure CPU, cheap) ────────────────────
    std::vector<float> blob = cpuPreprocess(image); // TODO: FIX , WHY ARE WE DOING THIS ON CPU

    // ── Allocate per-call context ─────────────────────────────────────────────
    auto ctx = std::make_unique<InferenceContext>();

    CUDA_CHECK(cudaStreamCreateWithFlags(&ctx->stream, cudaStreamNonBlocking));
    CUDA_CHECK(cudaMalloc(&ctx->d_input,  input_size_bytes_));
    CUDA_CHECK(cudaMalloc(&ctx->d_output, output_size_bytes_));
    ctx->h_output.resize(output_size_bytes_ / sizeof(float));

    // Grab the future before we move ctx into CallbackData
    auto future = ctx->promise.get_future();

    // ── H2D async ─────────────────────────────────────────────────────────────
    CUDA_CHECK(cudaMemcpyAsync(
        ctx->d_input,
        blob.data(),
        input_size_bytes_,
        cudaMemcpyHostToDevice,
        ctx->stream));

    // ── TRT inference async ───────────────────────────────────────────────────
    // enqueueV2 submits work to the stream and returns immediately.
    // IExecutionContext is safe to call from multiple threads provided each
    // call uses a distinct CUDA stream — which we guarantee here.
    void* bindings[2];
    bindings[input_binding_idx_]  = ctx->d_input;
    bindings[output_binding_idx_] = ctx->d_output;

    if (!context_->enqueueV2(bindings, ctx->stream, nullptr))
        throw std::runtime_error("TRT enqueueV2 failed");

    // ── D2H async ─────────────────────────────────────────────────────────────
    CUDA_CHECK(cudaMemcpyAsync(
        ctx->h_output.data(),
        ctx->d_output,
        output_size_bytes_,
        cudaMemcpyDeviceToHost,
        ctx->stream));

    // ── Schedule callback ─────────────────────────────────────────────────────
    // CallbackData takes ownership of ctx. The raw pointer is passed through
    // the C-style callback API and re-owned inside onInferenceComplete.
    auto* cbd  = new CallbackData{std::move(ctx), this, scale_x, scale_y, std::move(image)};

    CUDA_CHECK(cudaStreamAddCallback(
        cbd->ctx->stream,
        &Yolo11TensorRTModel::onInferenceComplete,
        cbd,
        0 /*flags — must be 0*/));

    // ── Return immediately ────────────────────────────────────────────────────
    // The caller can do other work and call future.get() whenever it needs
    // the result. If inference has already completed by then, get() returns
    // instantly; otherwise it blocks until onInferenceComplete fires.
    return future;
}

// ─── Class name loading ───────────────────────────────────────────────────────

void Yolo11TensorRTModel::initializeClassNames()
{
    const std::string path = package_share_dir_ + "/models/class_names.txt";
    std::ifstream file(path);

    if (!file.is_open()) {
        RCLCPP_WARN(logger_,
                    "Cannot open class names file '%s' — using COCO defaults",
                    path.c_str());
        class_names_ = {
            "person","bicycle","car","motorcycle","airplane","bus","train","truck",
            "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
            "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
            "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
            "skis","snowboard","sports ball","kite","baseball bat","baseball glove",
            "skateboard","surfboard","tennis racket","bottle","wine glass","cup",
            "fork","knife","spoon","bowl","banana","apple","sandwich","orange",
            "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
            "potted plant","bed","dining table","toilet","tv","laptop","mouse",
            "remote","keyboard","cell phone","microwave","oven","toaster","sink",
            "refrigerator","book","clock","vase","scissors","teddy bear",
            "hair drier","toothbrush"
        };
        return;
    }

    class_names_.clear();
    std::string line;
    while (std::getline(file, line)) {
        // Strip trailing whitespace / CR
        const auto last = line.find_last_not_of(" \t\r\n");
        if (last != std::string::npos)
            class_names_.push_back(line.substr(0, last + 1));
    }
    RCLCPP_INFO(logger_, "Loaded %zu class names from '%s'",
                class_names_.size(), path.c_str());
}