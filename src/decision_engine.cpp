#include "decision_engine.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/stat.h>

namespace {
const char* mode_name(DriveMode mode) {
    switch (mode) {
    case DriveMode::IDLE: return "IDLE";
    case DriveMode::FOLLOW: return "FOLLOW";
    case DriveMode::AUTOPILOT: return "AUTOPILOT";
    case DriveMode::MANUAL: return "MANUAL";
    }
    return "UNKNOWN";
}
}

DecisionEngine::DecisionEngine(const PipelineConfig&       cfg,
                               SafeQueue<PerceptionResult>& perception_queue,
                               SafeQueue<DriveCommand>&     command_queue,
                               RCReceiver&                  rc_receiver,
                               const std::string&           pipe_path)
    : cfg_(cfg), perception_queue_(perception_queue),
      command_queue_(command_queue), rc_receiver_(rc_receiver), pipe_path_(pipe_path),
      servo_(cfg.servo_gpio_pin),
      pi_throttle_controller_(cfg, PIDAxis::THROTTLE, cfg.pi_kp, cfg.pi_ki),
      pd_steer_controller_(cfg, PIDAxis::STEERING, cfg.pd_kp, 0.0f, cfg.pd_kd),
      vfh_controller_(cfg) {}

DecisionEngine::~DecisionEngine() { stop(); }

void DecisionEngine::start() {
    running_ = true;
    pipe_thread_     = std::thread(&DecisionEngine::pipe_reader_loop, this);
    decision_thread_ = std::thread(&DecisionEngine::decision_loop, this);
}

void DecisionEngine::stop() {
    running_ = false;
    if (pipe_thread_.joinable())     pipe_thread_.join();
    if (decision_thread_.joinable()) decision_thread_.join();
}

void DecisionEngine::pipe_reader_loop() {
    struct stat st;
    if (stat(pipe_path_.c_str(), &st) != 0) {
        if (mkfifo(pipe_path_.c_str(), 0666) != 0) {
            fprintf(stderr, "[T3] Failed to create FIFO %s\n", pipe_path_.c_str());
            return;
        }
    }

    fprintf(stdout, "[T3] Waiting for wake-word writer on %s...\n", pipe_path_.c_str());

    int fd = -1;
    auto open_pipe = [&]() -> bool {
        if (fd >= 0) close(fd);
        fd = open(pipe_path_.c_str(), O_RDONLY | O_NONBLOCK);
        return fd >= 0;
    };

    if (!open_pipe()) {
        fprintf(stderr, "[T3] Failed to open FIFO: %s\n", strerror(errno));
        return;
    }
    fprintf(stdout, "[T3] Wake-word pipe ready\n");

    char buf[256];
    std::string leftover;

    while (running_) {
        struct pollfd pfd = { fd, POLLIN, 0 };
        int ret = poll(&pfd, 1, 200);
        if (ret <= 0) continue;

        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n <= 0) {
            if (n == 0) {
                fprintf(stdout, "[T3] Pipe writer disconnected, reopening...\n");
                if (!open_pipe()) break;
                leftover.clear();
                fprintf(stdout, "[T3] Wake-word pipe reconnected\n");
            }
            continue;
        }

        buf[n] = '\0';
        leftover += buf;

        size_t pos;
        while ((pos = leftover.find('\n')) != std::string::npos) {
            std::string keyword = leftover.substr(0, pos);
            leftover.erase(0, pos + 1);

            if (keyword == "follow_me") {
                mode_.store(DriveMode::FOLLOW);
                servo_.setAngle(cfg_.angle_follow_me_mode);
                fprintf(stdout, "[T3] Wake word: '%s' -> mode FOLLOW\n", keyword.c_str());
            } 
            
            else if (keyword == "autopilot") {
                mode_.store(DriveMode::AUTOPILOT);
                servo_.setAngle(cfg_.angle_autopilot_mode);
                fprintf(stdout, "[T3] Wake word: '%s' -> mode AUTOPILOT\n", keyword.c_str());
            } 

            else if (keyword == "stop_engine") {
                mode_.store(DriveMode::IDLE);
                command_queue_.push(DriveCommand{0.0f, 0.0f});
                fprintf(stdout, "[T3] Wake word: '%s' -> mode IDLE\n", keyword.c_str());
            }
            
            else if (keyword == "three_sixty") {
                action_queue_.push(Action::SPIN_360);
                fprintf(stdout, "[T3] Wake word: '%s' -> action SPIN_360\n", keyword.c_str());
            } 

            else if (keyword == "turn_around") {
                action_queue_.push(Action::TURN_180);
                fprintf(stdout, "[T3] Wake word: '%s' -> action TURN_180\n", keyword.c_str());
            } 
            
            else if (keyword == "full_stop") {
                fprintf(stdout, "[T3] Wake word: '%s' -> shutting down\n", keyword.c_str());
                raise(SIGINT);
            } 
        }
    }

    if (fd >= 0) close(fd);
    fprintf(stdout, "[T3] Pipe reader stopped\n");
}

void DecisionEngine::decision_loop() {
    DriveMode switch_mode = DriveMode::IDLE;
    fprintf(stdout, "[T3] Decision engine started in IDLE\n");
    while (running_) {
        Action action;
        while (action_queue_.pop(action, 0)) {
            execute_action(action);
        }

        // The RC switch, whenever it has a fresh signal, is the sole authority
        // over AUTOPILOT/FOLLOW/MANUAL — it overrides whatever voice control
        // last set. Without a live RC link the switch position can't be
        // trusted, so mode falls back to voice control (this also covers
        // running with no receiver attached at all, e.g. voice-only testing).
        if (rc_receiver_.signal_fresh(cfg_.rc_signal_timeout_ms)) {
            if  (rc_receiver_.activate_switch_reading()) {
                switch_mode = rc_receiver_.selected_drive_mode();
            }

            if (switch_mode != DriveMode::MANUAL) {
                manual_reentry_blocked_ = false;  // switch left MANUAL: re-arm it
            } else if (manual_reentry_blocked_) {
                // Was manually driven, the link dropped, and the switch is
                // still sitting at MANUAL — refuse to resume until it's
                // physically cycled off that position first.
                switch_mode = DriveMode::IDLE;
            }

            if (switch_mode != mode_.load()) {
                fprintf(stdout, "[T3] RC switch -> mode %s\n", mode_name(switch_mode));
                if (switch_mode == DriveMode::FOLLOW)    servo_.setAngle(cfg_.angle_follow_me_mode);
                if (switch_mode == DriveMode::AUTOPILOT) servo_.setAngle(cfg_.angle_autopilot_mode);
            }
            mode_.store(switch_mode);
        } else if (mode_.load() == DriveMode::MANUAL) {
            fprintf(stderr, "[T3] RC signal lost while in MANUAL, mode -> IDLE\n");
            mode_.store(DriveMode::IDLE);
            manual_reentry_blocked_ = true;
        }

        DriveMode m = mode_.load();

        if (m == DriveMode::MANUAL) {
            // Keep observing detection events, without waiting for inference
            // or changing the transmitter's command cadence.
            PerceptionResult observation;
            if (perception_queue_.pop(observation, 0)) log_person_event(observation, m);
            command_queue_.push(rc_receiver_.get_drive_command());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if (m == DriveMode::IDLE) {
            PerceptionResult observation;
            if (perception_queue_.pop(observation, 0)) log_person_event(observation, m);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        PerceptionResult result;
        if (!perception_queue_.pop(result, 200)) {
            command_queue_.push(DriveCommand{0.0f, 0.0f});
            continue;
        }

        log_person_event(result, m);

        DriveCommand cmd{0.0f, 0.0f};
        float steer = 0.0f, throttle = 0.0f;

        switch (m) {
        case DriveMode::FOLLOW:
            // Follow the largest tracked person: PD centers them in frame
            // (steering), PI holds the preset stand-off distance (throttle).
            steer = pd_steer_controller_.compute_control(result);
            throttle = pi_throttle_controller_.compute_control(result);
            cmd = {throttle, steer};
            break;
        case DriveMode::AUTOPILOT:
            // Drives off the depth map alone (no person tracking): VFH+
            // picks a steering direction from the polar obstacle histogram,
            // biased toward straight-ahead; throttle is proportional to
            // that direction's own clearance, eased off in turns.
            cmd = vfh_controller_.compute_control(result);
            break;
        default:
            break;
        }

        command_queue_.push(cmd);
    }

    command_queue_.push(DriveCommand{0.0f, 0.0f});
    fprintf(stdout, "[T3] Decision engine stopped\n");
}

void DecisionEngine::log_person_event(const PerceptionResult& result, DriveMode mode) {
    PersonEvent event = person_tracker_.update(result);
    if (event.count > 0) {
        const auto& person = event.target;
        fprintf(stdout,
                "[T3] Person frame in %s: count=%d, target_score=%.2f, "
                "target_center=(%d,%d), target_box=(%d,%d,%d,%d), target_depth=%.4f raw\n",
                mode_name(mode), event.count, person.score,
                (person.x1 + person.x2) / 2, (person.y1 + person.y2) / 2,
                person.x1, person.y1, person.x2, person.y2, person.depth);
    } else {
        fprintf(stdout, "[T3] Person frame in %s: count=0\n", mode_name(mode));
    }

    if (event.kind == PersonEvent::Kind::DETECTED) {
        const auto& person = event.target;
        fprintf(stdout,
                "[T3] Person detected in %s: %d person(s), score=%.2f, center=(%d,%d), depth=%.3f raw\n",
                mode_name(mode), event.count, person.score,
                (person.x1 + person.x2) / 2, (person.y1 + person.y2) / 2,
                person.depth);
    } else if (event.kind == PersonEvent::Kind::LOST) {
        fprintf(stdout, "[T3] Person no longer detected in %s\n", mode_name(mode));
    }
}

void DecisionEngine::execute_action(Action action) {
    DriveCommand cmd{0.0f, 0.0f};
    int duration_ms = 0;

    switch (action) {
    case Action::SPIN_360:
        fprintf(stdout, "[T3] Executing SPIN_360\n");
        cmd = {0.0f, 1.0f};
        duration_ms = cfg_.spin_360_duration_ms;
        break;
    case Action::TURN_180:
        fprintf(stdout, "[T3] Executing TURN_180\n");
        cmd = {0.0f, 1.0f};
        duration_ms = cfg_.turn_180_duration_ms;
        break;
    }

    auto start = std::chrono::steady_clock::now();
    while (running_) {
        PerceptionResult observation;
        if (perception_queue_.pop(observation, 0)) {
            log_person_event(observation, mode_.load());
        }
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) break;
        command_queue_.push(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    command_queue_.push(DriveCommand{0.0f, 0.0f});
    fprintf(stdout, "[T3] Action complete\n");
}
