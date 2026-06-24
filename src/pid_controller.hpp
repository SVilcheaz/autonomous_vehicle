#pragma once

#include "types.hpp"
#include "config.hpp"
#include "perception_result.hpp"

class PIDController {
public:
    PIDController(const PipelineConfig& cfg, float kp, float ki = 0.0f, float kd = 0.0f);
    ~PIDController();

    DriveCommand compute_follow(const PerceptionResult& result);

private:
    float kp_{}, ki_{}, kd_{};
    int camera_width_;
};