#pragma once

#include <atomic>
#include <thread>
#include <opencv2/opencv.hpp>
#include "safe_queue.hpp"

class CameraCapture {
public:
    CameraCapture(int width, int height, SafeQueue<cv::Mat>& frame_queue);
    ~CameraCapture();

    void start();
    void stop();

    bool is_running() const { return running_.load(); }

private:
    void capture_loop();

    int                   width_;
    int                   height_;
    SafeQueue<cv::Mat>&   frame_queue_;
    std::atomic<bool>     running_{false};
    std::thread           thread_;
};
