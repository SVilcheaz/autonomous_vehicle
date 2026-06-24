#pragma once

enum class DriveMode { IDLE, FOLLOW, AUTOPILOT };

enum class Action { SPIN_360, TURN_180 };

struct DriveCommand {
    float throttle = 0.0f;  // 0..1
    float steering = 0.0f;  // -1..1
};
