#include <lgpio.h>
#include <cstdio>
#include <algorithm>
#include <array>
#include <cmath>

#include "config.hpp"

namespace {

struct Motor {
    const char* name;
    int  ena, in1, in2;
    bool invert;  // true => physical "forward" is negative raw speed
};

void set_motor(int handle, const Motor& m, float speed) {
    if (m.invert) speed = -speed;

    int duty = static_cast<int>(std::clamp(std::abs(speed), 0.0f, 1.0f) * 100.0f);

    if (speed > 0.0f) {
        lgGpioWrite(handle, m.in1, 1);
        lgGpioWrite(handle, m.in2, 0);
    } else if (speed < 0.0f) {
        lgGpioWrite(handle, m.in1, 0);
        lgGpioWrite(handle, m.in2, 1);
    } else {
        lgGpioWrite(handle, m.in1, 0);
        lgGpioWrite(handle, m.in2, 0);
    }
    lgTxPwm(handle, m.ena, 1000, duty, 0, 0);
}

} // namespace

int main() {
    const PipelineConfig cfg;

    // Forward directions match MotorController (src/motor_controller.cpp),
    // derived from hardware testing on this chassis.
    const std::array<Motor, 4> motors = {{
        {"Left Front",  cfg.left_front_ena,  cfg.left_front_in1,  cfg.left_front_in2,  true},
        {"Left Rear",   cfg.left_rear_enb,   cfg.left_rear_in3,   cfg.left_rear_in4,   false},
        {"Right Front", cfg.right_front_ena, cfg.right_front_in1, cfg.right_front_in2, false},
        {"Right Rear",  cfg.right_rear_enb,  cfg.right_rear_in3,  cfg.right_rear_in4,  true},
    }};

    int handle = lgGpiochipOpen(cfg.gpio_chip);
    if (handle < 0) {
        fprintf(stderr, "[motor_test] Cannot open gpiochip%d\n", cfg.gpio_chip);
        return 1;
    }

    for (const auto& m : motors) {
        lgGpioClaimOutput(handle, 0, m.ena, 0);
        lgGpioClaimOutput(handle, 0, m.in1, 0);
        lgGpioClaimOutput(handle, 0, m.in2, 0);
    }

    constexpr float  kTestSpeed    = 0.5;   // 50% duty
    constexpr double kSpinSeconds  = 2.0;
    constexpr double kPauseSeconds = 1.0;

    for (const auto& m : motors) {
        fprintf(stdout, "[motor_test] %s forward for %.0fs...\n", m.name, kSpinSeconds);
        set_motor(handle, m, kTestSpeed);
        lguSleep(kSpinSeconds);

        set_motor(handle, m, 0.0f);
        lguSleep(kPauseSeconds);
    }

    fprintf(stdout, "[motor_test] Done.\n");
    for (const auto& m : motors) set_motor(handle, m, 0.0f);
    lgGpiochipClose(handle);
    return 0;
}
