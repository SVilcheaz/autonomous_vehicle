#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>

#include "perception_result.hpp"

// Records every frame that successfully completes both inference passes.
// A bounded queue applies backpressure instead of silently dropping frames.
class PipelineRecorder {
public:
    PipelineRecorder(std::filesystem::path directory, int frame_width, int frame_height);
    ~PipelineRecorder();

    bool start();
    bool enqueue(PerceptionResult result);
    void stop();
    bool failed() const { return failed_.load(); }

private:
    struct Frame {
        PerceptionResult result;
        int64_t processed_at_unix_ms;
    };

    void write_loop();
    cv::Mat render(const PerceptionResult& result, uint64_t frame_index) const;

    std::filesystem::path directory_;
    int frame_width_;
    int frame_height_;
    cv::VideoWriter video_;
    std::ofstream timestamps_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable has_frames_;
    std::condition_variable has_space_;
    std::deque<Frame> pending_;
    bool stopping_{false};
    std::atomic<bool> failed_{false};
    uint64_t written_frames_{0};
    bool finalized_{false};

    static constexpr size_t kQueueCapacity = 8;
    static constexpr int kDepthPanelWidth = 384;
    static constexpr double kPlaybackFps = 10.0;
};
