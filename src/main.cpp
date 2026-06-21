#include <csignal>
#include <atomic>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "safe_queue.hpp"
#include "camera_capture.hpp"
#include "inference_engine.hpp"
#include "decision_engine.hpp"
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

    // T3
    DecisionEngine decision(cfg, perception_queue);

    camera.start();
    engine.start();
    decision.start();

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    decision.stop();
    engine.stop();
    camera.stop();

    fprintf(stdout, "[Main] Done.\n");
    return 0;
}
