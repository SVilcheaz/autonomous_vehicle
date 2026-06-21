#pragma once

#include <atomic>
#include <thread>
#include <string>

#include "config.hpp"
#include "safe_queue.hpp"
#include "perception_result.hpp"

enum class DriveMode { IDLE, FOLLOW, AUTOPILOT, STOP };

enum class Action { SPIN_360 };

class DecisionEngine {
public:
    DecisionEngine(const PipelineConfig&        cfg,
                   SafeQueue<PerceptionResult>&  perception_queue,
                   const std::string&            pipe_path = "/tmp/wake_word_pipe");
    ~DecisionEngine();

    void start();
    void stop();

    DriveMode mode() const { return mode_.load(); }

private:
    void pipe_reader_loop();
    void decision_loop();
    void execute_action(Action action);

    const PipelineConfig&        cfg_;
    SafeQueue<PerceptionResult>& perception_queue_;
    std::string                  pipe_path_;

    std::atomic<bool>      running_{false};
    std::atomic<DriveMode> mode_{DriveMode::IDLE};
    SafeQueue<Action>      action_queue_{4};

    std::thread pipe_thread_;
    std::thread decision_thread_;
};
