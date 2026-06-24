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

DecisionEngine::DecisionEngine(const PipelineConfig&       cfg,
                               SafeQueue<PerceptionResult>& perception_queue,
                               SafeQueue<DriveCommand>&     command_queue,
                               const std::string&           pipe_path)
    : cfg_(cfg), perception_queue_(perception_queue),
      command_queue_(command_queue), pipe_path_(pipe_path),
      servo_(cfg.servo_gpio_pin), 
      pi_throttle_controller_(cfg, cfg.pi_kp, cfg.pi_ki),
      pd_steer_controller_(cfg, cfg.pd_kp, 0.0f, cfg.pd_kd) {}

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
    while (running_) {
        Action action;
        while (action_queue_.pop(action, 0)) {
            execute_action(action);
        }

        DriveMode m = mode_.load();
        if (m == DriveMode::IDLE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        PerceptionResult result;
        if (!perception_queue_.pop(result, 200)) {
            command_queue_.push(DriveCommand{0.0f, 0.0f});
            continue;
        }

        fprintf(stdout, "[T3] Frame (%zu detections), mode=%d\n",
                result.detections.size(), (int)m);

        DriveCommand cmd{0.0f, 0.0f};

        switch (m) {
        case DriveMode::FOLLOW:
            // TODO: compute throttle/steering from person tracking
            // Vector field histogram for navigation on depth map
            break;
        case DriveMode::AUTOPILOT:
            // TODO: compute throttle/steering from occupancy grid
            // PD controller for steering
            // PI controller for throttle
            break;
        default:
            break;
        }

        command_queue_.push(cmd);
    }

    command_queue_.push(DriveCommand{0.0f, 0.0f});
    fprintf(stdout, "[T3] Decision engine stopped\n");
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
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) break;
        command_queue_.push(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    command_queue_.push(DriveCommand{0.0f, 0.0f});
    fprintf(stdout, "[T3] Action complete\n");
}
