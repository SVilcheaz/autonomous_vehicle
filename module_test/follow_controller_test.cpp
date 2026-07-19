#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "config.hpp"
#include "perception_result.hpp"
#include "pid_controller.hpp"

namespace {

Detection make_person(int x1, int y1, int x2, int y2, float depth) {
    return Detection{"person", 0.9f, x1, y1, x2, y2, depth};
}

PerceptionResult make_result(std::vector<Detection> dets, int w, int h) {
    PerceptionResult r;
    r.detections = std::move(dets);
    r.frame_w    = w;
    r.frame_h    = h;
    return r;
}

// Fresh PD/PI pair per scenario (own integral/derivative state), matching
// how DecisionEngine owns one instance per axis for the whole session.
// Unlike VFHController's tick-based slew limiter, PIDController derives dt
// from the real wall clock (now_ms()), so ticks need a real sleep between
// them for the I/D terms to see meaningful elapsed time.
void run_scenario(const char* name, const PipelineConfig& cfg,
                  const PerceptionResult& result, int ticks, int sleep_ms) {
    PIDController steer(cfg, PIDAxis::STEERING, cfg.pd_kp, 0.0f, cfg.pd_kd);
    PIDController throttle(cfg, PIDAxis::THROTTLE, cfg.pi_kp, cfg.pi_ki);

    for (int i = 0; i < ticks; ++i) {
        float s = steer.compute_control(result);
        float t = throttle.compute_control(result);
        fprintf(stdout, "[follow_test] %-38s tick=%d -> throttle=%+.3f steering=%+.3f\n",
                name, i, t, s);
        if (i + 1 < ticks) std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}

} // namespace

int main() {
    PipelineConfig cfg;
    // Config ships with these at 0 (bench-tuned on the real robot) — set
    // representative gains so this test actually exercises the control law.
    cfg.pd_kp = 0.8f;
    cfg.pd_kd = 0.15f;
    cfg.pi_kp = 1.5f;
    cfg.pi_ki = 0.3f;
    cfg.follow_target_depth = 0.5f;
    cfg.follow_invert_depth = false;  // larger raw value = farther, for this test

    const int W = cfg.camera_width;   // 1280
    const int H = cfg.camera_height;  // 720
    const int half_w = W / 2;

    // 1. Person centered, at target depth: both axes should sit near zero.
    {
        auto result = make_result(
            {make_person(half_w - 50, 200, half_w + 50, 600, cfg.follow_target_depth)}, W, H);
        run_scenario("person centered, at target depth", cfg, result, 3, 50);
    }

    // 2. Person off to the left, at target depth: steering only.
    {
        auto result = make_result(
            {make_person(0, 200, 100, 600, cfg.follow_target_depth)}, W, H);
        run_scenario("person left of frame", cfg, result, 3, 50);
    }

    // 3. Person centered but too far: throttle should ramp up as the I term
    //    accumulates over ticks.
    {
        auto result = make_result(
            {make_person(half_w - 50, 200, half_w + 50, 600, cfg.follow_target_depth + 0.3f)}, W, H);
        run_scenario("person centered, too far", cfg, result, 6, 100);
    }

    // 4. Person centered but too close: throttle negative (back off).
    {
        auto result = make_result(
            {make_person(half_w - 50, 200, half_w + 50, 600, cfg.follow_target_depth - 0.3f)}, W, H);
        run_scenario("person centered, too close", cfg, result, 3, 50);
    }

    // 5. Sustained large error: output should saturate at +1 and stay
    //    pinned there rather than run away (anti-windup clamp inside
    //    PIDController::compute_control). Custom gains here so saturation
    //    is visible within a handful of ticks.
    {
        auto result = make_result(
            {make_person(half_w - 50, 200, half_w + 50, 600, cfg.follow_target_depth + 1.0f)}, W, H);
        PIDController throttle(cfg, PIDAxis::THROTTLE, 0.5f, 1.0f);
        const int ticks = 10;
        for (int i = 0; i < ticks; ++i) {
            float t = throttle.compute_control(result);
            fprintf(stdout, "[follow_test] %-38s tick=%d -> throttle=%+.3f\n",
                    "sustained far (anti-windup)", i, t);
            if (i + 1 < ticks) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 6. No person in frame (a non-"person" detection is present but
    //    ignored): both axes should stay at zero.
    {
        std::vector<Detection> dets = {Detection{"car", 0.95f, 300, 300, 500, 450, 0.4f}};
        auto result = make_result(dets, W, H);
        run_scenario("no person in frame", cfg, result, 2, 50);
    }

    // 7. Multiple people: the largest bbox (find_target's area comparison)
    //    should win over a smaller, off-target, wrong-depth one.
    {
        std::vector<Detection> dets = {
            make_person(half_w - 100, 100, half_w + 100, 700, cfg.follow_target_depth), // big, centered, at target
            make_person(1100, 50, 1160, 110, 0.9f),                                     // small, off to the side, wrong depth
        };
        auto result = make_result(dets, W, H);
        run_scenario("multiple people, largest wins", cfg, result, 3, 50);
    }

    fprintf(stdout, "[follow_test] Done.\n");
    return 0;
}
