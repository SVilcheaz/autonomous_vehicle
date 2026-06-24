#pragma once

#include <atomic>
#include <thread>
#include <string>

#include "types.hpp"
#include "config.hpp"
#include "safe_queue.hpp"
#include "perception_result.hpp"
#include "servo_controller.hpp"
#include "pid_controller.hpp"

class DecisionEngine {
public:
    DecisionEngine(const PipelineConfig&         cfg,
                   SafeQueue<PerceptionResult>&  perception_queue,
                   SafeQueue<DriveCommand>&      command_queue,
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
    SafeQueue<DriveCommand>&     command_queue_;
    std::string                  pipe_path_;

    std::atomic<bool>      running_{false};
    std::atomic<DriveMode> mode_{DriveMode::IDLE};
    SafeQueue<Action>      action_queue_{4};

    ServoController servo_;

    PIDController pd_steer_controller_;
    PIDController pi_throttle_controller_;

    std::thread pipe_thread_;
    std::thread decision_thread_;
};
