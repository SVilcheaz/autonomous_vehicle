#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct Detection {
    std::string label;
    float       score;
    int         x1, y1, x2, y2;
    float       depth;  // raw model units at bbox centroid — not calibrated to metres
};

struct PerceptionResult {
    std::vector<Detection> detections;
    cv::Mat                depth_map;  // CV_32FC1, model output resolution
    cv::Mat                frame;      // original BGR camera frame
    int                    frame_w;
    int                    frame_h;
};
