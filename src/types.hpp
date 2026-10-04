#pragma once
#include <chrono>
#include <cstdint>

enum class DriveMode { IDLE, FOLLOW, AUTOPILOT, MANUAL };

enum class Action { SPIN_360, TURN_180 };

struct DriveCommand {
    float throttle = 0.0f;  // -1..1 negative value mean reverse
    float steering = 0.0f;  // -1..1
};

inline int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
