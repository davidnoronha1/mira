#pragma once

#include "../../app.hpp"

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <future>

// ─── TRT deleter helpers ──────────────────────────────────────────────────────

struct TRTDestroy {
    template <class T>
    void operator()(T* obj) const noexcept { if (obj) obj->destroy(); }
};

template <class T>
using TRTUniquePtr = std::unique_ptr<T, TRTDestroy>;

// ─── TRT logger bridging to RCLCPP ───────────────────────────────────────────

class TRTLogger : public nvinfer1::ILogger {
public:
    explicit TRTLogger(rclcpp::Logger ros_logger)
        : ros_logger_(ros_logger) {}

    void log(Severity severity, const char* msg) noexcept override;

private:
    rclcpp::Logger ros_logger_;
};

// ─── Per-call inference context ───────────────────────────────────────────────
// Owns all resources for one in-flight inference so calls never share state.

struct InferenceContext {
    cudaStream_t   stream   = nullptr;
    void*          d_input  = nullptr;
    void*          d_output = nullptr;
    std::vector<float> h_output;
    std::promise<std::pair<std::vector<Detection>, cv::Mat>> promise;

    InferenceContext() = default;

    // Non-copyable, non-movable — always heap-allocated via unique_ptr
    InferenceContext(const InferenceContext&)            = delete;
    InferenceContext& operator=(const InferenceContext&) = delete;

    ~InferenceContext() {
        if (d_input)  cudaFree(d_input);
        if (d_output) cudaFree(d_output);
        if (stream)   cudaStreamDestroy(stream);
    }
};

// ─── Main class ───────────────────────────────────────────────────────────────

class Yolo11TensorRTModel : public ImageProcessor {
public:
    Yolo11TensorRTModel(rclcpp::Logger     logger,
                        const std::string& device,
                        int                height,
                        int                width);

    ~Yolo11TensorRTModel() override = default;

    // Non-copyable
    Yolo11TensorRTModel(const Yolo11TensorRTModel&)            = delete;
    Yolo11TensorRTModel& operator=(const Yolo11TensorRTModel&) = delete;

    // Returns a future that resolves once GPU inference + postprocess complete.
    // Multiple calls may be in-flight simultaneously on separate CUDA streams.
    std::future<std::pair<std::vector<Detection>, cv::Mat>> processImage(const cv::Mat& image) override;

private:
    // ── Model constants ───────────────────────────────────────────────────────
    static constexpr int   MODEL_INPUT_W   = 640;
    static constexpr int   MODEL_INPUT_H   = 640;
    static constexpr float CONF_THRESHOLD  = 0.25f;
    static constexpr float NMS_THRESHOLD   = 0.45f;

    // ── TRT objects ───────────────────────────────────────────────────────────
    std::unique_ptr<TRTLogger>               trt_logger_;
    TRTUniquePtr<nvinfer1::IRuntime>         runtime_;
    TRTUniquePtr<nvinfer1::ICudaEngine>      engine_;
    TRTUniquePtr<nvinfer1::IExecutionContext> context_;

    // ── Binding metadata (set once during init) ───────────────────────────────
    int    input_binding_idx_      = -1;
    int    output_binding_idx_     = -1;
    size_t input_size_bytes_       = 0;
    size_t output_size_bytes_      = 0;
    int    output_num_values_      = 0;   // 4 + num_classes
    int    output_num_detections_  = 0;   // num anchors (e.g. 8400)

    // ── Misc ─────────────────────────────────────────────────────────────────
    std::string              package_share_dir_;
    std::vector<std::string> class_names_;

    // ── Init helpers ─────────────────────────────────────────────────────────
    void loadEngine(const std::string& engine_path);
    void resolveBindings();
    void initializeClassNames();

    // ── Per-inference helpers ─────────────────────────────────────────────────
    std::vector<float> cpuPreprocess(const cv::Mat& frame) const;

    std::vector<Detection> postProcess(const float* output_data,
                                       int          num_detections,
                                       int          num_classes_plus4,
                                       float        conf_threshold,
                                       float        nms_threshold,
                                       float        scale_x,
                                       float        scale_y) const;

    // ── CUDA stream callback (static so it can be passed as C function ptr) ───
    struct CallbackData {
        std::unique_ptr<InferenceContext> ctx;
        Yolo11TensorRTModel*              self;
        float                             scale_x;
        float                             scale_y;
        cv::Mat                           image;
    };

    void CUDART_CB onInferenceComplete(cudaStream_t  stream,
                                              cudaError_t   status,
                                              void*         userdata);
};