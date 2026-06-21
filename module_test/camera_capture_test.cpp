#include <cstdio>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "camera_capture.hpp"

static const std::string OUTPUT_DIR = "results/camera_test";

int main() {
    std::filesystem::create_directories(OUTPUT_DIR);

    SafeQueue<cv::Mat> frame_queue(1);
    CameraCapture cam(1280, 720, frame_queue);

    fprintf(stdout, "[camera_test] Starting camera...\n");
    cam.start();

    cv::Mat frame;
    if (!frame_queue.pop(frame, 5000)) {
        fprintf(stderr, "[camera_test] Timed out waiting for frame\n");
        cam.stop();
        return 1;
    }
    cam.stop();

    std::string path = OUTPUT_DIR + "/photo.jpg";
    if (!cv::imwrite(path, frame)) {
        fprintf(stderr, "[camera_test] Failed to write %s\n", path.c_str());
        return 1;
    }

    fprintf(stdout, "[camera_test] Saved %dx%d frame to %s\n",
            frame.cols, frame.rows, path.c_str());
    return 0;
}