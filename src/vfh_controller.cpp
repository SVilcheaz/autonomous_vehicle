#include "vfh_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

VFHController::VFHController(const PipelineConfig& cfg)
    : cfg_(cfg),
      num_sectors_(std::max(1, cfg.vfh_num_sectors)),
      prev_blocked_(num_sectors_, false) {}

bool VFHController::too_close(float raw, float thresh) const {
    return cfg_.follow_invert_depth ? (raw > thresh) : (raw < thresh);
}

void VFHController::build_polar_histogram(const cv::Mat& depth_map,
                                          std::vector<float>& out) const {
    const int h = depth_map.rows;
    const int w = depth_map.cols;

    const int r0 = std::clamp((int)(cfg_.vfh_row_top_frac    * h), 0, h - 1);
    const int r1 = std::clamp((int)(cfg_.vfh_row_bottom_frac * h), r0 + 1, h);

    for (int i = 0; i < num_sectors_; ++i) {
        int c0 = (int)((int64_t)i       * w / num_sectors_);
        int c1 = (int)((int64_t)(i + 1) * w / num_sectors_);
        c1 = std::clamp(c1, c0 + 1, w);

        cv::Rect roi(c0, r0, c1 - c0, r1 - r0);
        double minVal, maxVal;
        cv::minMaxLoc(depth_map(roi), &minVal, &maxVal);
        // Nearest obstacle in the sector: smallest raw value if larger means
        // farther, largest raw value if the model reports inverse depth.
        out[i] = cfg_.follow_invert_depth ? (float)maxVal : (float)minVal;
    }
}

int VFHController::select_direction(const std::vector<bool>& blocked,
                                    const std::vector<float>& dist) const {
    const int goal_sector = num_sectors_ / 2;  // straight ahead

    int best_sector    = -1;
    int best_goal_diff = std::numeric_limits<int>::max();

    int i = 0;
    while (i < num_sectors_) {
        if (blocked[i]) { ++i; continue; }

        int start = i;
        while (i < num_sectors_ && !blocked[i]) ++i;
        int end   = i;              // free run is [start, end)
        int width = end - start;

        int candidate;
        if (width >= cfg_.vfh_smax_sectors) {
            // Wide valley: steer to the border nearest straight-ahead,
            // offset inward so we keep a smax/2-sector safety margin from
            // the obstacle edge rather than hugging it.
            int margin = cfg_.vfh_smax_sectors / 2;
            if (goal_sector <= start)          candidate = start + margin;
            else if (goal_sector >= end - 1)   candidate = end - 1 - margin;
            else                                candidate = goal_sector;
            candidate = std::clamp(candidate, start, end - 1);
        } else {
            // Narrow valley: not enough room to offset from center safely.
            candidate = start + width / 2;
        }

        int goal_diff = std::abs(candidate - goal_sector);
        if (goal_diff < best_goal_diff) {
            best_goal_diff = goal_diff;
            best_sector    = candidate;
        }
    }

    if (best_sector >= 0) return best_sector;

    // Every sector blocked: steer toward the single least-bad (farthest
    // nearest-obstacle reading) direction. Throttle will fall out near zero
    // on its own via the clearance ratio below.
    int fallback = 0;
    for (int j = 1; j < num_sectors_; ++j) {
        if (!too_close(dist[j], dist[fallback])) fallback = j;
    }
    return fallback;
}

float VFHController::sector_to_steering(int sector) const {
    float norm = ((sector + 0.5f) / (float)num_sectors_) * 2.0f - 1.0f;
    return std::clamp(norm, -1.0f, 1.0f);
}

DriveCommand VFHController::compute_control(const PerceptionResult& result) {
    if (result.depth_map.empty()) return DriveCommand{0.0f, 0.0f};

    std::vector<float> sector_dist(num_sectors_);
    build_polar_histogram(result.depth_map, sector_dist);

    std::vector<bool> blocked(num_sectors_);
    for (int i = 0; i < num_sectors_; ++i) {
        // Hysteresis: a sector already blocked needs to clear the farther
        // vfh_obstacle_clear_dist_m threshold to become free again, instead
        // of flickering right at obstacle_dist_m.
        float thresh = prev_blocked_[i] ? cfg_.vfh_obstacle_clear_dist_m
                                         : cfg_.obstacle_dist_m;
        blocked[i] = too_close(sector_dist[i], thresh);
    }
    prev_blocked_ = blocked;

    int selected = select_direction(blocked, sector_dist);

    // Slew-rate limit the steering output — stands in for VFH+'s usual
    // previous-direction cost term to damp oscillation between valleys.
    float target_steer = sector_to_steering(selected);
    float delta = std::clamp(target_steer - prev_steering_,
                             -cfg_.vfh_max_steer_rate, cfg_.vfh_max_steer_rate);
    float steering = prev_steering_ + delta;
    prev_steering_ = steering;

    // Throttle: proportional to clearance in the chosen direction, eased
    // off in turns. No separate PI loop — there's no fixed distance to
    // regulate toward in open-road driving.
    float denom = cfg_.vfh_full_speed_dist_m - cfg_.obstacle_dist_m;
    float speed_norm = (std::abs(denom) > 1e-6f)
        ? std::clamp((sector_dist[selected] - cfg_.obstacle_dist_m) / denom, 0.0f, 1.0f)
        : 1.0f;
    float turn_factor = std::clamp(1.0f - cfg_.vfh_turn_slowdown * std::abs(steering), 0.0f, 1.0f);
    float throttle = speed_norm * turn_factor;

    return DriveCommand{throttle, steering};
}
