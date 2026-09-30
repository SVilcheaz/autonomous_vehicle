#include "pipeline_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace {
int64_t unix_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
}

PipelineRecorder::PipelineRecorder(std::filesystem::path directory,
                                   int frame_width, int frame_height)
    : directory_(std::move(directory)),
      frame_width_(frame_width), frame_height_(frame_height) {}

PipelineRecorder::~PipelineRecorder() { stop(); }

bool PipelineRecorder::start() {
    const auto video_path = directory_ / "annotated.mp4";
    const auto timestamps_path = directory_ / "frames.csv";

    // avc1 uses OpenCV's FFmpeg H.264 encoder on the target Raspberry Pi.
    video_.open(video_path.string(), cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                kPlaybackFps, {frame_width_ + kDepthPanelWidth, frame_height_});
    if (!video_.isOpened()) {
        fprintf(stderr, "[Record] Cannot open H.264 video: %s\n", video_path.c_str());
        failed_ = true;
        return false;
    }

    timestamps_.open(timestamps_path);
    if (!timestamps_) {
        fprintf(stderr, "[Record] Cannot open timestamps: %s\n", timestamps_path.c_str());
        video_.release();
        failed_ = true;
        return false;
    }
    timestamps_ << "frame_index,processed_at_unix_ms\n";

    thread_ = std::thread(&PipelineRecorder::write_loop, this);
    fprintf(stdout, "[Record] Saving every processed frame to %s (playback %.0f fps)\n",
            video_path.c_str(), kPlaybackFps);
    return true;
}

bool PipelineRecorder::enqueue(PerceptionResult result) {
    const int64_t processed_at_unix_ms = unix_ms();
    std::unique_lock<std::mutex> lock(mutex_);
    has_space_.wait(lock, [this] {
        return pending_.size() < kQueueCapacity || stopping_ || failed_;
    });
    if (stopping_ || failed_) return false;
    pending_.push_back({std::move(result), processed_at_unix_ms});
    has_frames_.notify_one();
    return true;
}

void PipelineRecorder::stop() {
    if (finalized_) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    has_frames_.notify_all();
    has_space_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (video_.isOpened()) video_.release();
    if (timestamps_.is_open()) {
        timestamps_.flush();
        if (!timestamps_) {
            fprintf(stderr, "[Record] Failed to finish timestamps file\n");
            failed_ = true;
        }
        timestamps_.close();
    }
    if (written_frames_ > 0) {
        cv::VideoCapture check((directory_ / "annotated.mp4").string());
        auto frames_in_file = static_cast<uint64_t>(
            check.get(cv::CAP_PROP_FRAME_COUNT));
        if (!check.isOpened() || frames_in_file != written_frames_) {
            fprintf(stderr, "[Record] Video frame count mismatch: wrote %llu, file has %llu\n",
                    (unsigned long long)written_frames_,
                    (unsigned long long)frames_in_file);
            failed_ = true;
        }
    }
    finalized_ = true;
}

void PipelineRecorder::write_loop() {
    uint64_t frame_index = 0;
    try {
        for (;;) {
            Frame frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                has_frames_.wait(lock, [this] { return !pending_.empty() || stopping_; });
                if (pending_.empty()) break;
                frame = std::move(pending_.front());
                pending_.pop_front();
                has_space_.notify_one();
            }

            video_.write(render(frame.result, frame_index));
            timestamps_ << frame_index << ',' << frame.processed_at_unix_ms << '\n';
            if (!timestamps_) throw std::runtime_error("timestamps write failed");
            ++frame_index;
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[Record] Recording failed: %s\n", e.what());
        failed_ = true;
        has_space_.notify_all();
    }
    written_frames_ = frame_index;
    fprintf(stdout, "[Record] Wrote %llu frames\n", (unsigned long long)frame_index);
}

cv::Mat PipelineRecorder::render(const PerceptionResult& result,
                                 uint64_t frame_index) const {
    cv::Mat canvas(frame_height_, frame_width_ + kDepthPanelWidth,
                   CV_8UC3, cv::Scalar(24, 24, 24));
    cv::Mat camera = canvas(cv::Rect(0, 0, frame_width_, frame_height_));
    result.frame.copyTo(camera);

    for (const auto& det : result.detections) {
        const cv::Scalar color(70, 230, 70);
        cv::rectangle(camera, {det.x1, det.y1}, {det.x2, det.y2}, color, 2);
        char label[160];
        snprintf(label, sizeof(label), "%s %.0f%% depth %.3f",
                 det.label.c_str(), det.score * 100.0f, det.depth);
        int baseline = 0;
        cv::Size size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                        0.55, 1, &baseline);
        int x = std::clamp(det.x1, 0, std::max(0, frame_width_ - size.width - 8));
        int y = std::clamp(det.y1, size.height + 8, frame_height_ - 6);
        cv::rectangle(camera, {x, y - size.height - 7},
                      {x + size.width + 8, y + baseline + 2}, color, cv::FILLED);
        cv::putText(camera, label, {x + 4, y - 2},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 1);
    }

    const std::string title = "Frame " + std::to_string(frame_index);
    cv::putText(camera, title, {16, 32}, cv::FONT_HERSHEY_SIMPLEX,
                0.85, {0, 0, 0}, 4);
    cv::putText(camera, title, {16, 32}, cv::FONT_HERSHEY_SIMPLEX,
                0.85, {255, 255, 255}, 2);

    const int panel_x = frame_width_;
    cv::putText(canvas, "Depth (relative)", {panel_x + 12, 36},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 2);
    if (!result.depth_map.empty()) {
        cv::Mat scaled, colored;
        cv::normalize(result.depth_map, scaled, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(scaled, colored, cv::COLORMAP_MAGMA);
        const int side = std::min(kDepthPanelWidth - 16, frame_height_ - 120);
        cv::Mat resized;
        cv::resize(colored, resized, {side, side});
        resized.copyTo(canvas(cv::Rect(panel_x + (kDepthPanelWidth - side) / 2,
                                       65, side, side)));

        double min_depth = 0.0, max_depth = 0.0;
        cv::minMaxLoc(result.depth_map, &min_depth, &max_depth);
        char range[100];
        snprintf(range, sizeof(range), "Raw range: %.3f to %.3f",
                 min_depth, max_depth);
        cv::putText(canvas, range, {panel_x + 12, 85 + side},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1);
    }
    return canvas;
}
