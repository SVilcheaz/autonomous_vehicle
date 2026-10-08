#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "follow_controller.hpp"

namespace {
void check(bool condition, const char* message) {
    if (!condition) {
        fprintf(stderr, "[follow_search_test] FAIL: %s\n", message);
        std::exit(1);
    }
}

bool same_command(DriveCommand actual, DriveCommand expected) {
    return std::abs(actual.throttle - expected.throttle) < 1e-5f &&
           std::abs(actual.steering - expected.steering) < 1e-5f;
}

PerceptionResult clear_scene() {
    PerceptionResult result{};
    result.frame_w = 1280;
    result.frame_h = 720;
    result.depth_map = cv::Mat(48, 64, CV_32FC1, cv::Scalar(5.0f));
    return result;
}
}

int main() {
    PipelineConfig cfg;
    cfg.follow_invert_depth = false;
    cfg.follow_target_depth = 3.0f;
    cfg.pi_kp = 0.5f;
    cfg.pi_ki = 0.8f;
    cfg.pd_kp = 0.7f;
    cfg.pd_kd = 0.1f;

    VFHController autopilot(cfg);
    FollowController follow(cfg, autopilot);
    auto empty = clear_scene();
    auto person = clear_scene();
    person.detections.push_back({"person", 0.9f, 540, 100, 740, 650, 3.0f});

    auto command = follow.compute_control(person);
    check(!follow.searching(), "visible person uses FOLLOW");
    check(same_command(command, {0.0f, 0.0f}), "hold centered person at target distance");

    VFHController expected_autopilot(cfg);
    for (int i = 0; i < 4; ++i) {
        command = follow.compute_control(empty);
        check(follow.searching(), "no person starts and maintains search");
        check(same_command(command, expected_autopilot.compute_control(empty)),
              "search uses the existing AUTOPILOT commands");
        check(command.throttle > 0.0f, "search drives when the path is clear");
    }

    auto car = empty;
    car.detections.push_back({"car", 0.95f, 0, 0, 100, 100, 3.0f});
    follow.compute_control(car);
    check(follow.searching(), "a non-person detection does not resume FOLLOW");

    command = follow.compute_control(person);
    check(!follow.searching(), "first person detection resumes FOLLOW");
    check(same_command(command, {0.0f, 0.0f}), "reacquisition stops at target distance");

    // Build up integral/derivative history, lose the person, then acquire
    // a different person. Compare with a fresh controller at the same target.
    auto far_left = person;
    far_left.detections[0] = {"person", 0.9f, 0, 100, 200, 650, 4.0f};
    follow.compute_control(far_left);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    follow.compute_control(far_left);
    follow.compute_control(empty);
    auto new_person = person;
    new_person.detections[0] = {"person", 0.9f, 1000, 100, 1200, 650, 3.5f};
    // A smaller person at a different location/distance must not be selected.
    new_person.detections.push_back({"person", 0.9f, 0, 0, 50, 50, 2.0f});
    VFHController fresh_autopilot(cfg);
    FollowController fresh_follow(cfg, fresh_autopilot);
    command = follow.compute_control(new_person);
    check(!follow.searching(), "any person can end search");
    check(same_command(command, fresh_follow.compute_control(new_person)),
          "reacquisition discards accumulated PID state");
    check(command.throttle > 0.0f && command.steering > 0.0f,
          "largest person is followed after reacquisition");

    auto no_depth = empty;
    no_depth.depth_map.release();
    command = follow.compute_control(no_depth);
    check(follow.searching(), "missing depth does not pretend to find a person");
    check(same_command(command, {0.0f, 0.0f}), "search without depth stops");
    command = follow.compute_control(person);
    check(!follow.searching() && same_command(command, {0.0f, 0.0f}),
          "a person resumes FOLLOW after missing depth");

    auto blocked = empty;
    blocked.depth_map = cv::Mat(48, 64, CV_32FC1, cv::Scalar(0.3f));
    command = follow.compute_control(blocked);
    check(command.throttle == 0.0f, "blocked search does not drive forward");
    follow.reset();
    check(!follow.searching(), "leaving FOLLOW cancels search state");
    command = follow.compute_control(person);
    check(same_command(command, {0.0f, 0.0f}), "reentering FOLLOW starts with reset PID state");
    follow.reset();
    command = follow.compute_control(empty);
    check(follow.searching() && command.throttle > 0.0f,
          "selecting FOLLOW without a person starts search");

    fprintf(stdout, "[follow_search_test] search, reacquisition, PID reset, and depth handling verified\n");
    return 0;
}
