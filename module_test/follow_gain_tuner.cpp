#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

#include "config.hpp"
#include "perception_result.hpp"
#include "pid_controller.hpp"

// Closed-loop gain sweep for FOLLOW's two PIDController axes. Rather than
// hand-picking kp/ki/kd by trial and error on the real robot, this drives
// the *actual* PIDController class against a small synthetic plant (see
// simulate_steering/simulate_throttle) and ranks candidate gains by a
// step-response score. Uses PIDController exactly as DecisionEngine does —
// no changes to pid_controller.hpp/.cpp — so PIDController's dt comes from
// the real wall clock, meaning this sweep takes roughly a minute of actual
// sleeping. That's the tradeoff for not touching production code.
//
// The plant models are simplified (steering/throttle set the RATE of
// change of apparent position/depth, i.e. instantaneous turn/accel
// response — no motor lag or inertia), so treat the winning gains as a
// starting point, not a final answer: confirm with follow_controller_test
// and do a short tuning pass on the real robot afterward, same as any
// other bench-calibrated value in config.hpp.

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

struct AxisTrace {
    std::vector<float> t, err, out;
};

struct StepMetrics {
    float rise_time_s     = -1.0f;  // time to first reach within 10% of the initial error
    float settling_time_s = -1.0f;  // time after which |error| stays within tolerance for good
    float overshoot       = 0.0f;   // peak |error| after first crossing zero
    float final_error     = 0.0f;
    bool  settled          = false;
};

StepMetrics analyze(const AxisTrace& trace, float tolerance) {
    StepMetrics m;
    if (trace.err.empty()) return m;
    m.final_error = trace.err.back();

    const float initial  = trace.err.front();
    const float abs_init = std::abs(initial);
    bool crossed = false;

    for (size_t i = 0; i < trace.err.size(); ++i) {
        if (m.rise_time_s < 0.0f && abs_init > 1e-6f &&
            std::abs(trace.err[i]) <= 0.1f * abs_init) {
            m.rise_time_s = trace.t[i];
        }
        if (!crossed && ((initial > 0.0f && trace.err[i] < 0.0f) ||
                         (initial < 0.0f && trace.err[i] > 0.0f))) {
            crossed = true;
        }
        if (crossed) m.overshoot = std::max(m.overshoot, std::abs(trace.err[i]));
    }

    int last_outside = -1;
    for (size_t i = 0; i < trace.err.size(); ++i) {
        if (std::abs(trace.err[i]) > tolerance) last_outside = (int)i;
    }
    if (last_outside < 0) {
        m.settling_time_s = trace.t.front();
        m.settled = true;
    } else if (last_outside < (int)trace.err.size() - 1) {
        m.settling_time_s = trace.t[last_outside + 1];
        m.settled = true;
    }
    return m;
}

// Lower is better. Unsettled runs are penalised heavily so they always
// sort last; overshoot is weighted fairly heavily too — a fast response
// that blows past the target isn't actually good on a physical robot.
float score(const StepMetrics& m, float duration_s) {
    float settle = m.settled ? m.settling_time_s : duration_s * 10.0f;
    return settle + 2.0f * m.overshoot;
}

AxisTrace simulate_steering(const PipelineConfig& cfg, float kp, float kd,
                            float initial_error, float dt, float duration_s,
                            float plant_gain_frac_per_s) {
    PIDController ctrl(cfg, PIDAxis::STEERING, kp, 0.0f, kd);
    const int   W      = cfg.camera_width;
    const int   H      = cfg.camera_height;
    const float half_w = W / 2.0f;
    float cx = half_w + initial_error * half_w;

    AxisTrace trace;
    const int steps = std::max(1, (int)(duration_s / dt));
    for (int i = 0; i <= steps; ++i) {
        auto result = make_result(
            {make_person((int)(cx - 50), 200, (int)(cx + 50), 600, cfg.follow_target_depth)}, W, H);
        float steering = ctrl.compute_control(result);
        float error     = (cx - half_w) / half_w;

        trace.t.push_back(i * dt);
        trace.err.push_back(error);
        trace.out.push_back(steering);

        cx = std::clamp(cx - plant_gain_frac_per_s * half_w * steering * dt, 0.0f, (float)W);
        if (i < steps) std::this_thread::sleep_for(std::chrono::duration<float>(dt));
    }
    return trace;
}

AxisTrace simulate_throttle(const PipelineConfig& cfg, float kp, float ki,
                            float initial_error, float dt, float duration_s,
                            float plant_gain_units_per_s) {
    PIDController ctrl(cfg, PIDAxis::THROTTLE, kp, ki, 0.0f);
    const int W = cfg.camera_width;
    const int H = cfg.camera_height;
    float depth = cfg.follow_target_depth + initial_error;

    AxisTrace trace;
    const int steps = std::max(1, (int)(duration_s / dt));
    for (int i = 0; i <= steps; ++i) {
        auto result = make_result(
            {make_person(W / 2 - 50, 200, W / 2 + 50, 600, depth)}, W, H);
        float throttle = ctrl.compute_control(result);
        float error     = depth - cfg.follow_target_depth;

        trace.t.push_back(i * dt);
        trace.err.push_back(error);
        trace.out.push_back(throttle);

        depth -= plant_gain_units_per_s * throttle * dt;
        if (i < steps) std::this_thread::sleep_for(std::chrono::duration<float>(dt));
    }
    return trace;
}

void print_trace(const char* name, const AxisTrace& trace, int stride) {
    for (size_t i = 0; i < trace.t.size(); i += stride) {
        fprintf(stdout, "[gain_tuner]   %-8s t=%5.2fs error=%+.3f output=%+.3f\n",
                name, trace.t[i], trace.err[i], trace.out[i]);
    }
}

template <typename SimulateFn>
void run_sweep(const char* axis_name, const char* gain1_name, const char* gain2_name,
              const std::vector<float>& gain1_values, const std::vector<float>& gain2_values,
              float duration_s, float tolerance, SimulateFn simulate) {
    struct Candidate {
        float      gain1, gain2, score;
        AxisTrace  trace;
        StepMetrics metrics;
    };
    std::vector<Candidate> candidates;

    for (float g1 : gain1_values) {
        for (float g2 : gain2_values) {
            AxisTrace trace = simulate(g1, g2);
            StepMetrics m   = analyze(trace, tolerance);
            candidates.push_back({g1, g2, score(m, duration_s), std::move(trace), m});
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.score < b.score; });

    fprintf(stdout, "[gain_tuner]   %-6s %-6s | rise_s settle_s overshoot final_err | score\n",
            gain1_name, gain2_name);
    for (const auto& c : candidates) {
        fprintf(stdout, "[gain_tuner]   %-6.2f %-6.2f |  %5.2f   %5.2f    %6.3f    %+6.3f   | %6.3f%s\n",
                c.gain1, c.gain2, c.metrics.rise_time_s,
                c.metrics.settled ? c.metrics.settling_time_s : -1.0f,
                c.metrics.overshoot, c.metrics.final_error, c.score,
                (&c == &candidates.front()) ? "  <- best" : "");
    }

    const auto& best = candidates.front();
    fprintf(stdout, "[gain_tuner] Best %s trajectory (%s=%.2f %s=%.2f):\n",
            axis_name, gain1_name, best.gain1, gain2_name, best.gain2);
    print_trace(axis_name, best.trace, 4);
}

} // namespace

int main() {
    PipelineConfig cfg;
    const float dt = 0.05f;

    fprintf(stdout, "[gain_tuner] Sweeping FOLLOW's PD/PI gains against a simplified "
                    "closed-loop plant.\n[gain_tuner] Uses the real PIDController and its "
                    "real-time clock, so this takes roughly a minute.\n\n");

    // ── Steering (PD): person starts at the left edge of frame ──────────
    {
        const float duration_s  = 3.0f;
        const float initial_err = -0.9f;
        const float tolerance   = 0.05f;
        // Rough estimate: full-lock steering sweeps the tracked point across
        // ~2x the half-frame width per second (a skid-steer rover can pivot
        // fast). Adjust once the real robot's turn rate is known.
        const float plant_gain  = 2.0f;

        fprintf(stdout, "[gain_tuner] Steering (PD) sweep - initial error=%+.2f tolerance=+-%.2f duration=%.1fs\n",
                initial_err, tolerance, duration_s);
        run_sweep("steer", "kp", "kd",
                  {0.4f, 0.7f, 1.0f, 1.5f}, {0.0f, 0.1f, 0.2f},
                  duration_s, tolerance,
                  [&](float kp, float kd) {
                      return simulate_steering(cfg, kp, kd, initial_err, dt, duration_s, plant_gain);
                  });
    }

    fprintf(stdout, "\n");

    // ── Throttle (PI): person starts too far away ────────────────────────
    {
        const float duration_s  = 4.0f;
        const float initial_err = 0.3f;  // depth units too far
        const float tolerance   = 0.02f;
        // Rough estimate: full throttle closes ~0.3 depth-units/second.
        // Adjust once the depth model's raw-unit scale is known.
        const float plant_gain  = 0.3f;

        fprintf(stdout, "[gain_tuner] Throttle (PI) sweep - initial error=%+.2f tolerance=+-%.2f duration=%.1fs\n",
                initial_err, tolerance, duration_s);
        run_sweep("throttle", "kp", "ki",
                  {0.5f, 1.5f, 3.0f}, {0.0f, 0.3f, 0.6f},
                  duration_s, tolerance,
                  [&](float kp, float ki) {
                      return simulate_throttle(cfg, kp, ki, initial_err, dt, duration_s, plant_gain);
                  });
    }

    fprintf(stdout, "[gain_tuner] Done.\n");
    return 0;
}
