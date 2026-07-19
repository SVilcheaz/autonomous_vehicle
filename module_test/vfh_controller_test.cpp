#include <cstdio>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "perception_result.hpp"
#include "vfh_controller.hpp"

namespace {

PerceptionResult make_result(const cv::Mat& depth_map) {
    PerceptionResult r;
    r.depth_map = depth_map;
    r.frame_w   = depth_map.cols;
    r.frame_h   = depth_map.rows;
    return r;
}

// A fresh VFHController per scenario (own hysteresis/slew state). Runs
// enough ticks for the steering slew-rate limiter to settle on its target
// before printing the final DriveCommand.
void run_scenario(const char* name, const PipelineConfig& cfg, const cv::Mat& depth_map) {
    VFHController vfh(cfg);
    PerceptionResult result = make_result(depth_map);

    DriveCommand cmd{0.0f, 0.0f};
    for (int i = 0; i < 20; ++i) {
        cmd = vfh.compute_control(result);
    }
    fprintf(stdout, "[vfh_test] %-32s -> throttle=%+.3f steering=%+.3f\n",
            name, cmd.throttle, cmd.steering);
}

} // namespace

int main() {
    PipelineConfig cfg;
    cfg.follow_invert_depth = false;  // larger raw value = farther, for this test

    const int W = 64, H = 48;

    // 1. All clear: expect steering ~0, throttle high.
    {
        cv::Mat depth(H, W, CV_32FC1, cv::Scalar(5.0f));
        run_scenario("all clear", cfg, depth);
    }

    // 2. Obstacle dead ahead, both sides open: expect steering off-center,
    //    throttle still high (chosen direction itself is clear).
    {
        cv::Mat depth(H, W, CV_32FC1, cv::Scalar(5.0f));
        depth(cv::Rect(W / 3, 0, W / 3, H)).setTo(0.5f);
        run_scenario("obstacle ahead, sides clear", cfg, depth);
    }

    // 3. Obstacle spans almost the whole frame, narrow gap far right:
    //    expect steering saturated toward the gap, throttle reduced by the
    //    turn (but not zero — the gap itself reads clear).
    {
        cv::Mat depth(H, W, CV_32FC1, cv::Scalar(0.5f));
        depth(cv::Rect(W - W / 12, 0, W / 12, H)).setTo(5.0f);
        run_scenario("narrow gap on right", cfg, depth);
    }

    // 4. Blocked everywhere, right half less close than left: expect
    //    throttle ~0, steering toward the least-bad (right) fallback.
    {
        cv::Mat depth(H, W, CV_32FC1, cv::Scalar(0.3f));
        depth(cv::Rect(W / 2, 0, W / 2, H)).setTo(0.5f);
        run_scenario("fully blocked (right less bad)", cfg, depth);
    }

    fprintf(stdout, "[vfh_test] Done.\n");
    return 0;
}
