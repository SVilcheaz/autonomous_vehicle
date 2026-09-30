#pragma once

#include <atomic>
#include <thread>
#include <cstdint>
#include <mutex>

#include "types.hpp"
#include "config.hpp"
#include "safe_queue.hpp"

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
    bool apply(const DriveCommand& cmd, bool resuming);
    void set_motor(const Motor& m, float speed);
    void all_stop();

    const PipelineConfig& cfg_;
    SafeQueue<DriveCommand>& command_queue_;

    std::atomic<bool>    running_{false};
    std::atomic<int64_t> last_cmd_epoch_ms_{0};
    std::mutex           gpio_mutex_;
    bool                 output_active_{false};  // guarded by gpio_mutex_

    // Only the control thread updates these log-throttling fields.
    DriveCommand last_logged_command_{};
    int64_t      last_log_ms_{0};
    bool         has_logged_command_{false};
    bool         last_logged_motion_{false};

    int gpio_handle_{-1};

    Motor left_front_{}, left_rear_{}, right_front_{}, right_rear_{};

    std::thread control_thread_;
    std::thread watchdog_thread_;
};
