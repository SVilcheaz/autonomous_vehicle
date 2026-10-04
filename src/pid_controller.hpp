#pragma once

#include <cstdint>

#include "types.hpp"
#include "config.hpp"
#include "perception_result.hpp"

enum class PIDAxis { STEERING, THROTTLE };

class PIDController {
public:
    PIDController(const PipelineConfig& cfg, PIDAxis axis,
                   float kp, float ki = 0.0f, float kd = 0.0f);
    ~PIDController();

    float compute_control(const PerceptionResult& result);

private:
    bool find_target(const PerceptionResult& result, Detection& out) const;
    float compute_error(const Detection& target, int frame_w) const;

    const PipelineConfig& cfg_;
    PIDAxis axis_;
    float kp_{}, ki_{}, kd_{};

    float   integral_{0.0f};
    float   prev_error_{0.0f};
    int64_t prev_time_ms_{0};
    bool    had_target_{false};
};
