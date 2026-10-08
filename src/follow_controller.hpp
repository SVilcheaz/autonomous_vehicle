#pragma once

#include <algorithm>

#include "pid_controller.hpp"
#include "vfh_controller.hpp"

// FOLLOW remains the selected mode while its controller uses AUTOPILOT
// to search. This lets automatic reacquisition work with the RC switch
// held on FOLLOW, without changing explicit AUTOPILOT behavior.
class FollowController {
public:
    FollowController(const PipelineConfig& cfg, VFHController& autopilot)
        : steering_(cfg, PIDAxis::STEERING, cfg.pd_kp, 0.0f, cfg.pd_kd),
          throttle_(cfg, PIDAxis::THROTTLE, cfg.pi_kp, cfg.pi_ki),
          autopilot_(autopilot) {}

    DriveCommand compute_control(const PerceptionResult& result) {
        searching_ = std::none_of(result.detections.begin(), result.detections.end(),
            [](const Detection& detection) { return detection.label == "person"; });
        if (searching_) {
            steering_.reset();
            throttle_.reset();
            return autopilot_.compute_control(result);
        }
        return {throttle_.compute_control(result), steering_.compute_control(result)};
    }

    bool searching() const { return searching_; }

    void reset() {
        searching_ = false;
        steering_.reset();
        throttle_.reset();
    }

private:
    PIDController steering_;
    PIDController throttle_;
    VFHController& autopilot_;
    bool searching_{false};
};
