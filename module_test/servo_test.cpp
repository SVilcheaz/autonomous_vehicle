#include <cstdio>
#include <chrono>
#include <thread>
#include "servo_controller.hpp"
#include "config.hpp"

int main() {
    const PipelineConfig cfg;
    ServoController servo(cfg.servo_gpio_pin);

    fprintf(stdout, "[servo_test] Moving to autopilot position (%.1f deg)...\n",
            cfg.angle_autopilot_mode);
    servo.setAngle(cfg.angle_autopilot_mode);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    fprintf(stdout, "[servo_test] Moving to follow-me position (%.1f deg)...\n",
            cfg.angle_follow_me_mode);
    servo.setAngle(cfg.angle_follow_me_mode);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    fprintf(stdout, "[servo_test] Moving back to autopilot position (%.1f deg)...\n",
            cfg.angle_autopilot_mode);
    servo.setAngle(cfg.angle_autopilot_mode);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    fprintf(stdout, "[servo_test] Done.\n");
    return 0;
}