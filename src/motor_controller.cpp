#include "motor_controller.hpp"

#include <lgpio.h>
#include <cstdio>
#include <algorithm>
#include <chrono>
#include <cmath>

MotorController::MotorController(const PipelineConfig& cfg,
                                 SafeQueue<DriveCommand>& command_queue)
    : cfg_(cfg), command_queue_(command_queue) {}

MotorController::~MotorController() { stop(); }

bool MotorController::init() {
    // Direction invert flags derived from hardware testing (motor_driver.cpp):
    //   left front:  forward at negative raw speed
    //   left rear:   forward at positive raw speed
    //   right front: forward at positive raw speed
    //   right rear:  forward at negative raw speed
    left_front_  = {cfg_.left_front_ena,  cfg_.left_front_in1,  cfg_.left_front_in2,  true};
    left_rear_   = {cfg_.left_rear_enb,   cfg_.left_rear_in3,   cfg_.left_rear_in4,   false};
    right_front_ = {cfg_.right_front_ena, cfg_.right_front_in1, cfg_.right_front_in2, false};
    right_rear_  = {cfg_.right_rear_enb,  cfg_.right_rear_in3,  cfg_.right_rear_in4,  true};

    gpio_handle_ = lgGpiochipOpen(cfg_.gpio_chip);
    if (gpio_handle_ < 0) {
        fprintf(stderr, "[T4] Cannot open gpiochip%d\n", cfg_.gpio_chip);
        return false;
    }

    for (const auto* m : {&left_front_, &left_rear_, &right_front_, &right_rear_}) {
        lgGpioClaimOutput(gpio_handle_, 0, m->ena, 0);
        lgGpioClaimOutput(gpio_handle_, 0, m->in1, 0);
        lgGpioClaimOutput(gpio_handle_, 0, m->in2, 0);
    }

    fprintf(stdout, "[T4] GPIO initialised on chip %d\n", cfg_.gpio_chip);
    return true;
}

void MotorController::start() {
    if (gpio_handle_ < 0) return;
    running_ = true;
    last_cmd_epoch_ms_.store(now_ms());
    control_thread_  = std::thread(&MotorController::control_loop, this);
    watchdog_thread_ = std::thread(&MotorController::watchdog_loop, this);
}

void MotorController::stop() {
    running_ = false;
    if (control_thread_.joinable())  control_thread_.join();
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
    if (gpio_handle_ >= 0) {
        all_stop();
        lgGpiochipClose(gpio_handle_);
        gpio_handle_ = -1;
    }
}

void MotorController::control_loop() {
    fprintf(stdout, "[T4] Motor controller started\n");

    while (running_) {
        DriveCommand cmd;
        if (!command_queue_.pop(cmd, 100)) continue;

        std::lock_guard<std::mutex> lock(gpio_mutex_);
        last_cmd_epoch_ms_.store(now_ms());
        output_active_ = apply(cmd, !output_active_);
    }

    {
        std::lock_guard<std::mutex> lock(gpio_mutex_);
        all_stop();
        output_active_ = false;
    }
    fprintf(stdout, "[T4] Motor controller stopped\n");
}

void MotorController::watchdog_loop() {
    fprintf(stdout, "[T4] Watchdog started (timeout %d ms)\n", cfg_.watchdog_timeout_ms);

    while (running_) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(cfg_.watchdog_timeout_ms / 2));

        std::lock_guard<std::mutex> lock(gpio_mutex_);
        int64_t elapsed = now_ms() - last_cmd_epoch_ms_.load();
        if (elapsed > cfg_.watchdog_timeout_ms && output_active_) {
            all_stop();
            output_active_ = false;
            fprintf(stderr, "[T4] Watchdog: no command for %lld ms, emergency stop\n",
                    (long long)elapsed);
        }
    }

    fprintf(stdout, "[T4] Watchdog stopped\n");
}

bool MotorController::apply(const DriveCommand& cmd, bool resuming) {
    float throttle = std::clamp(cmd.throttle, -1.0f, 1.0f) * cfg_.max_throttle;
    float steering = std::clamp(cmd.steering, -1.0f, 1.0f) * cfg_.max_steering;

    float left_speed  = std::clamp(throttle + steering, -1.0f, 1.0f);
    float right_speed = std::clamp(throttle - steering, -1.0f, 1.0f);

    set_motor(left_front_,  left_speed);
    set_motor(left_rear_,   left_speed);
    set_motor(right_front_, right_speed);
    set_motor(right_rear_,  right_speed);

    const bool moving = std::abs(left_speed) >= 0.01f ||
                        std::abs(right_speed) >= 0.01f;
    const int64_t now = now_ms();
    const bool state_changed = has_logged_command_ && moving != last_logged_motion_;
    const bool significant_change =
        std::abs(cmd.throttle - last_logged_command_.throttle) >= 0.1f ||
        std::abs(cmd.steering - last_logged_command_.steering) >= 0.1f;
    if ((moving && resuming) || state_changed ||
        (significant_change && now - last_log_ms_ >= 1000)) {
        fprintf(stdout, "[T4] throttle=%.2f steer=%.2f -> L=%.2f R=%.2f\n",
                cmd.throttle, cmd.steering, left_speed, right_speed);
        last_logged_command_ = cmd;
        last_logged_motion_ = moving;
        last_log_ms_ = now;
        has_logged_command_ = true;
    }
    return moving;
}

void MotorController::set_motor(const Motor& m, float speed) {
    if (m.invert) speed = -speed;

    int duty = static_cast<int>(std::clamp(std::abs(speed), 0.0f, 1.0f) * 100.0f);

    if (speed > 0.0f) {
        lgGpioWrite(gpio_handle_, m.in1, 1);
        lgGpioWrite(gpio_handle_, m.in2, 0);
    } else if (speed < 0.0f) {
        lgGpioWrite(gpio_handle_, m.in1, 0);
        lgGpioWrite(gpio_handle_, m.in2, 1);
    } else {
        lgGpioWrite(gpio_handle_, m.in1, 0);
        lgGpioWrite(gpio_handle_, m.in2, 0);
    }

    lgTxPwm(gpio_handle_, m.ena, 1000, duty, 0, 0);
}

void MotorController::all_stop() {
    if (gpio_handle_ < 0) return;

    for (const auto* m : {&left_front_, &left_rear_, &right_front_, &right_rear_}) {
        lgGpioWrite(gpio_handle_, m->in1, 0);
        lgGpioWrite(gpio_handle_, m->in2, 0);
        lgTxPwm(gpio_handle_, m->ena, 1000, 0, 0, 0);
    }
}
