#include "decision_engine.hpp"

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/stat.h>

DecisionEngine::DecisionEngine(const PipelineConfig&       cfg,
                               SafeQueue<PerceptionResult>& perception_queue,
                               const std::string&           pipe_path)
    : cfg_(cfg), perception_queue_(perception_queue), pipe_path_(pipe_path) {}

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

            DriveMode new_mode = DriveMode::IDLE;
            if      (keyword == "follow_me")   new_mode = DriveMode::FOLLOW;
            else if (keyword == "autopilot")   new_mode = DriveMode::AUTOPILOT;
            else if (keyword == "three_sixty") new_mode = DriveMode::SPIN_360;
            else if (keyword == "full_stop")   new_mode = DriveMode::STOP;

            mode_.store(new_mode);
            fprintf(stdout, "[T3] Wake word: '%s' -> mode %d\n",
                    keyword.c_str(), (int)new_mode);
        }
    }

    if (fd >= 0) close(fd);
    fprintf(stdout, "[T3] Pipe reader stopped\n");
}

void DecisionEngine::decision_loop() {
    while (running_) {
        PerceptionResult result;
        if (!perception_queue_.pop(result, 200)) continue;

        DriveMode m = mode_.load();
        fprintf(stdout, "[T3] Frame (%zu detections), mode=%d\n",
                result.detections.size(), (int)m);

        // TODO: act on result + mode, push commands to T4
    }

    fprintf(stdout, "[T3] Decision engine stopped\n");
}
