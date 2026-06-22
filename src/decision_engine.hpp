#pragma once

#include <atomic>
#include <thread>
#include <string>

#include "config.hpp"
#include "safe_queue.hpp"
#include "perception_result.hpp"
#include "servo_controller.hpp"

enum class DriveMode { IDLE, FOLLOW, AUTOPILOT };

enum class Action { SPIN_360, TURN_180 };

struct DriveCommand {
    float throttle = 0.0f;  // 0..1
    float steering = 0.0f;  // -1..1
};

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

    ServoController servo_;

    std::thread pipe_thread_;
    std::thread decision_thread_;
};
