#include <csignal>
#include <atomic>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "safe_queue.hpp"
#include "camera_capture.hpp"
#include "inference_engine.hpp"
#include "perception_result.hpp"

static std::atomic<bool> g_running{true};

static void on_signal(int) { g_running = false; }

int main() {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const PipelineConfig cfg;

    SafeQueue<cv::Mat>          frame_queue(cfg.frame_queue_size);
    SafeQueue<PerceptionResult> perception_queue(cfg.perception_queue_size);

    // T1
    CameraCapture camera(cfg.camera_width, cfg.camera_height, frame_queue);

    // T2
    InferenceEngine engine(cfg, frame_queue, perception_queue);
    if (!engine.init()) {
        fprintf(stderr, "[Main] Inference engine init failed\n");
        return 1;
    }

    camera.start();
    engine.start();

    int frame_count = 0;
    while (g_running) {
        PerceptionResult result;
        if (!perception_queue.pop(result, 200)) continue;

        ++frame_count;
        fprintf(stdout, "[Main] Frame %d — %zu detection(s)\n",
                frame_count, result.detections.size());
        for (const auto& det : result.detections) {
            fprintf(stdout, "  %-20s  conf=%.2f  depth=%.3f  box=[%d,%d,%d,%d]\n",
                    det.label.c_str(), det.score, det.depth,
                    det.x1, det.y1, det.x2, det.y2);
        }

        // T3 will pop from perception_queue here — stub for now
    }

    engine.stop();
    camera.stop();

    fprintf(stdout, "[Main] Done. Processed %d frames.\n", frame_count);
    return 0;
}
