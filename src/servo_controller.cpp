#include "servo_controller.hpp"

#include <lgpio.h>
#include <cstdio>
#include <chrono>
#include <thread>

static constexpr double PWM_FREQ  = 50.0;
static constexpr int    SETTLE_MS = 500;

static double angleToDuty(double angle) {
    return 2.5 + (angle / 180.0) * 10.0;
}

ServoController::ServoController(int gpio_pin) : handle_(-1), pin_(gpio_pin) {
    handle_ = lgGpiochipOpen(0);
    if (handle_ < 0) {
        fprintf(stderr, "[Servo] Failed to open gpiochip: %s\n",
                lguErrorText(handle_));
        return;
    }
    lgGpioClaimOutput(handle_, 0, pin_, 0);
}

ServoController::~ServoController() {
    if (handle_ >= 0) {
        lgTxPwm(handle_, pin_, 0, 0, 0, 0);
        lgGpiochipClose(handle_);
    }
}

void ServoController::setAngle(double angle) {
    if (handle_ < 0) return;
    lgTxPwm(handle_, pin_, PWM_FREQ, angleToDuty(angle), 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_MS));
    lgTxPwm(handle_, pin_, 0, 0, 0, 0);
}