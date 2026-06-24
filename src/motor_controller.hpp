#pragma once

#include <atomic>
#include <thread>
#include <cstdint>

#include "config.hpp"
#include "safe_queue.hpp"
#include "decision_engine.hpp"

class MotorController {
public:
    MotorController(const PipelineConfig& cfg,
                    SafeQueue<DriveCommand>& command_queue);
    ~MotorController();

    bool init();
    void start();
    void stop();

private:
    struct Motor {
        int ena, in1, in2;
        bool invert;
    };

    void control_loop();
    void watchdog_loop();
    void apply(const DriveCommand& cmd);
    void set_motor(const Motor& m, float speed);
    void all_stop();

    const PipelineConfig& cfg_;
    SafeQueue<DriveCommand>& command_queue_;

    std::atomic<bool>    running_{false};
    std::atomic<int64_t> last_cmd_epoch_ms_{0};

    int gpio_handle_{-1};

    Motor left_front_{}, left_rear_{}, right_front_{}, right_rear_{};

    std::thread control_thread_;
    std::thread watchdog_thread_;
};
