#include <csignal>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "safe_queue.hpp"
#include "camera_capture.hpp"
#include "inference_engine.hpp"
#include "perception_result.hpp"

static constexpr int    MAX_FRAMES      = 100;
static constexpr double MAX_SECONDS     = 10.0;
static constexpr float  DEPTH_BLEND     = 0.4f;

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running = false; }

static cv::Mat render(const PerceptionResult& result) {
    cv::Mat canvas = result.frame.clone();

    if (!result.depth_map.empty()) {
        cv::Mat depth_norm;
        cv::normalize(result.depth_map, depth_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::Mat depth_color, depth_resized;
        cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_MAGMA);
        cv::resize(depth_color, depth_resized, canvas.size());
        cv::addWeighted(canvas, 1.0f - DEPTH_BLEND, depth_resized, DEPTH_BLEND, 0, canvas);
    }

    for (const auto& det : result.detections) {
        const cv::Scalar green(0, 255, 0);
        cv::rectangle(canvas, {det.x1, det.y1}, {det.x2, det.y2}, green, 2);

        char buf[128];
        snprintf(buf, sizeof(buf), "%s %d%% d=%.2f",
                 det.label.c_str(), (int)(det.score * 100), det.depth);

        int baseline = 0;
        auto text_sz = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        int text_y = std::max(det.y1 - 6, text_sz.height + 2);

        cv::rectangle(canvas,
                      {det.x1, text_y - text_sz.height - 4},
                      {det.x1 + text_sz.width + 4, text_y + 4},
                      green, cv::FILLED);
        cv::putText(canvas, buf, {det.x1 + 2, text_y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 1);
    }

    return canvas;
}

int main() {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    namespace fs = std::filesystem;
    const fs::path out_dir = "results/pipeline_test";
    fs::create_directories(out_dir);

    const PipelineConfig cfg;

    SafeQueue<cv::Mat>          frame_queue(cfg.frame_queue_size);
    SafeQueue<PerceptionResult> perception_queue(cfg.perception_queue_size);

    CameraCapture   camera(cfg.camera_width, cfg.camera_height, frame_queue);
    InferenceEngine engine(cfg, frame_queue, perception_queue);

    if (!engine.init()) {
        fprintf(stderr, "[Test] Inference engine init failed\n");
        return 1;
    }

    camera.start();
    engine.start();

    fprintf(stdout, "[Test] Running for up to %d frames / %.0fs\n",
            MAX_FRAMES, MAX_SECONDS);

    const auto t_start = std::chrono::steady_clock::now();
    int frame_count = 0;

    while (g_running && frame_count < MAX_FRAMES) {
        double secs = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        if (secs >= MAX_SECONDS) break;

        PerceptionResult result;
        if (!perception_queue.pop(result, 200)) continue;

        ++frame_count;
        secs = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();

        cv::Mat canvas = render(result);

        char info[64];
        snprintf(info, sizeof(info), "Frame %d  t=%.1fs  dets=%zu",
                 frame_count, secs, result.detections.size());
        cv::putText(canvas, info, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 2);

        char path[256];
        snprintf(path, sizeof(path), "%s/frame_%04d.png",
                 out_dir.c_str(), frame_count);
        cv::imwrite(path, canvas);

        fprintf(stdout, "[Test] Frame %d — %zu det(s), t=%.1fs\n",
                frame_count, result.detections.size(), secs);
        for (const auto& det : result.detections) {
            fprintf(stdout, "  %-20s  conf=%.2f  depth=%.3f  box=[%d,%d,%d,%d]\n",
                    det.label.c_str(), det.score, det.depth,
                    det.x1, det.y1, det.x2, det.y2);
        }
    }

    engine.stop();
    camera.stop();

    fprintf(stdout, "[Test] Done. Saved %d frames to %s/\n",
            frame_count, out_dir.c_str());
    return 0;
}
