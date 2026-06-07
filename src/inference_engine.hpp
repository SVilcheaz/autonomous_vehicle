#pragma once

#include <atomic>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <hailo/hailort.h>
#include <hailo/hailort_common.hpp>
#include <hailo/vdevice.hpp>
#include <hailo/infer_model.hpp>

#include "config.hpp"
#include "safe_queue.hpp"
#include "perception_result.hpp"

class InferenceEngine {
public:
    InferenceEngine(const PipelineConfig&        cfg,
                    SafeQueue<cv::Mat>&           frame_queue,
                    SafeQueue<PerceptionResult>&  perception_queue);
    ~InferenceEngine();

    bool init();   // load both models onto Hailo; call once before start()
    void start();
    void stop();

private:
    void inference_loop();

    cv::Mat letterbox(const cv::Mat& src,
                      float& scale, int& pad_x, int& pad_y) const;

    std::vector<Detection> decode_nms(
        const float* buf, const hailo_nms_shape_t& shape,
        float scale, int pad_x, int pad_y,
        int orig_w, int orig_h) const;

    const PipelineConfig&        cfg_;
    SafeQueue<cv::Mat>&          frame_queue_;
    SafeQueue<PerceptionResult>& perception_queue_;
    std::atomic<bool>            running_{false};
    std::thread                  thread_;

    // Hailo handles — one VDevice shared; scheduler multiplexes both models
    std::unique_ptr<hailort::VDevice>              vdevice_;
    std::shared_ptr<hailort::InferModel>           yolo_model_;
    std::shared_ptr<hailort::InferModel>           depth_model_;
    std::shared_ptr<hailort::ConfiguredInferModel> yolo_configured_;
    std::shared_ptr<hailort::ConfiguredInferModel> depth_configured_;

    // Bindings reused across frames; output buffers bound once at init
    std::optional<hailort::ConfiguredInferModel::Bindings> yolo_bindings_;
    std::optional<hailort::ConfiguredInferModel::Bindings> depth_bindings_;

    // Pre-allocated output buffers
    std::vector<float> yolo_out_buf_;
    std::vector<float> depth_out_buf_;

    // Cached names, sizes and shapes (avoid repeated API calls in the hot loop)
    std::string yolo_in_name_,  yolo_out_name_;
    std::string depth_in_name_, depth_out_name_;
    size_t      yolo_in_size_,  depth_in_size_;
    int         depth_in_h_,    depth_in_w_;
    int         depth_out_h_,   depth_out_w_;
    hailo_nms_shape_t yolo_nms_shape_{};
};
