#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "pipeline_recorder.hpp"
#include "session_logger.hpp"

int main() {
    namespace fs = std::filesystem;
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    auto dir = fs::temp_directory_path() /
               ("pipeline_recorder_test_" + std::to_string(getpid()) + "_" +
                std::to_string(stamp));
    fs::create_directories(dir);

    SessionLogger logger;
    if (!logger.start(dir / "pipeline.log")) return 1;
    fprintf(stdout, "[T1] camera log probe\n");
    fprintf(stderr, "[T2] inference error probe\n");

    PipelineRecorder recorder(dir, 640, 360);
    if (!recorder.start()) return 1;
    for (int i = 0; i < 3; ++i) {
        PerceptionResult result;
        result.frame = cv::Mat(360, 640, CV_8UC3, cv::Scalar(30 + i * 30, 60, 90));
        result.depth_map = cv::Mat(48, 48, CV_32FC1);
        for (int y = 0; y < result.depth_map.rows; ++y) {
            for (int x = 0; x < result.depth_map.cols; ++x)
                result.depth_map.at<float>(y, x) = static_cast<float>(x + y);
        }
        result.detections.push_back({"person", 0.9f, 100, 60, 250, 300, 0.5f});
        result.frame_w = 640;
        result.frame_h = 360;
        if (!recorder.enqueue(std::move(result))) return 1;
    }
    recorder.stop();
    logger.stop();

    assert(!recorder.failed());
    assert(fs::file_size(dir / "annotated.mp4") > 0);
    cv::VideoCapture video((dir / "annotated.mp4").string());
    assert(video.isOpened());
    cv::Mat frame;
    int frames = 0;
    while (video.read(frame)) {
        assert(frame.cols == 1024 && frame.rows == 360);
        const cv::Vec3b box = frame.at<cv::Vec3b>(60, 100);
        assert(box[1] > box[0] + 30 && box[1] > box[2] + 30);
        const cv::Scalar depth_color = cv::mean(frame(cv::Rect(700, 100, 100, 100)));
        assert(depth_color[0] + depth_color[1] + depth_color[2] > 100);
        ++frames;
    }
    assert(frames == 3);

    std::ifstream csv(dir / "frames.csv");
    std::string line;
    int rows = 0;
    while (std::getline(csv, line)) ++rows;
    assert(rows == 4);

    std::ifstream log(dir / "pipeline.log");
    std::string contents((std::istreambuf_iterator<char>(log)),
                         std::istreambuf_iterator<char>());
    assert(contents.find("[T1] camera log probe") != std::string::npos);
    assert(contents.find("[T2] inference error probe") != std::string::npos);
    fprintf(stdout, "[recorder_test] 3 frames and both log streams verified in %s\n",
            dir.c_str());
    return 0;
}
