#include <csignal>
#include <atomic>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "safe_queue.hpp"
#include "camera_capture.hpp"

static std::atomic<bool> g_running{true};

static void on_signal(int) { g_running = false; }

int main() {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const PipelineConfig cfg;

    SafeQueue<cv::Mat> frame_queue(cfg.frame_queue_size);
    CameraCapture camera(cfg.camera_width, cfg.camera_height, frame_queue);

    camera.start();

    int frame_count = 0;
    while (g_running) {
        cv::Mat frame;
        if (!frame_queue.pop(frame, 200)) {
            // Timeout — no frame yet, check g_running and retry.
            continue;
        }
        fprintf(stdout, "[Main] Frame %d: %dx%d  queue_size=%zu\n",
                ++frame_count, frame.cols, frame.rows, frame_queue.size());

        // T2 will pop from frame_queue here — stub for now.
    }

    camera.stop();
    fprintf(stdout, "[Main] Done. Total frames received: %d\n", frame_count);
    return 0;
}
