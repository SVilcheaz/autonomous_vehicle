#include <cstdio>
#include <chrono>
#include <thread>
#include <string>
#include "servo_controller.hpp"
#include "config.hpp"

int main(int argc, char* argv[]) {
    const std::string final_mode = argc > 1 ? argv[1] : "autopilot";
    if (argc > 2 || (final_mode != "autopilot" && final_mode != "follow-me")) {
        fprintf(stderr, "Usage: %s [autopilot|follow-me]\n", argv[0]);
        return 1;
    }
    const bool end_in_follow_me = final_mode == "follow-me";
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

    const double final_angle = end_in_follow_me
            ? cfg.angle_follow_me_mode : cfg.angle_autopilot_mode;
    fprintf(stdout, "[servo_test] Moving to final %s position (%.1f deg)...\n",
            end_in_follow_me ? "follow-me" : "autopilot", final_angle);
    servo.setAngle(final_angle);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    fprintf(stdout, "[servo_test] Done.\n");
    return 0;
}
