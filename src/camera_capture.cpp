#include "camera_capture.hpp"
#include <cstdio>
#include <unistd.h>

CameraCapture::CameraCapture(int width, int height, SafeQueue<cv::Mat>& frame_queue)
    : width_(width), height_(height), frame_queue_(frame_queue) {}

CameraCapture::~CameraCapture() {
    stop();
}

void CameraCapture::start() {
    running_ = true;
    thread_ = std::thread(&CameraCapture::capture_loop, this);
}

void CameraCapture::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void CameraCapture::capture_loop() {
    // NV12 from libcamerasrc, convert to BGR for OpenCV consumers downstream.
    // drop=true and sync=false: appsink discards frames it can't deliver fast enough,
    // and does not wait for the pipeline clock — we do our own frame management.
    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=" + std::to_string(width_) +
        ",height="           + std::to_string(height_) +
        ",format=NV12 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=true sync=false";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        fprintf(stderr, "[T1] Failed to open GStreamer pipeline\n");
        running_ = false;
        return;
    }
    sleep(1);
    fprintf(stdout, "[T1] Camera capture started (%dx%d)\n", width_, height_);

    while (running_) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            fprintf(stderr, "[T1] Failed to grab frame — retrying\n");
            continue;
        }
        cv::rotate(frame, frame, cv::ROTATE_180);
        // push() drops the oldest frame internally if the queue is full,
        // so T2 always sees the freshest image.
        frame_queue_.push(std::move(frame));
    }

    cap.release();
    fprintf(stdout, "[T1] Camera capture stopped\n");
}
