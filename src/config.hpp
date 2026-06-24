#pragma once

#include <cstddef>
#include <string>

enum class DepthModel { FASTDEPTH, DEPTH_ANYTHING };

struct PipelineConfig {
    // ── Camera (T1) ───────────────────────────────────────────────────────────
    int    camera_width           = 1280;
    int    camera_height          = 720;
    size_t frame_queue_size       = 2;      // ring buffer slots between T1 and T2

    // ── Inference (T2) ────────────────────────────────────────────────────────
    std::string yolo_hef          = "models/hailo/yolov8s_h8l.hef";
    DepthModel  depth_model       = DepthModel::DEPTH_ANYTHING;
    int         yolo_input_size   = 640;    // YOLO letterbox target (pixels)
    float       conf_threshold    = 0.4f;
    float       nms_iou_threshold = 0.45f;
    int         infer_timeout_ms  = 1000;
    size_t      perception_queue_size = 2;  // ring buffer slots between T2 and T3

    const char* depth_hef() const {
        return depth_model == DepthModel::FASTDEPTH
            ? "models/hailo/fastdepth--224x224.hef"
            : "models/hailo/depth_anything_v2--224x224.hef";
    }

    // ── Decision (T3) ────────────────────────────────────────────────────────
    float  obstacle_dist_m        = 1.0f;   // repulsion kicks in below this depth
    size_t command_queue_size     = 2;      // ring buffer slots between T3 and T4
    int    servo_gpio_pin         = 25;     // camera tilt servo
    double angle_follow_me_mode   = 0.0;   // degrees
    double angle_autopilot_mode   = 70.0;    // degrees

    // ── Actuation (T4) ───────────────────────────────────────────────────────
    int    gpio_chip              = 4;

    int    left_front_ena         = 19;
    int    left_front_in1         = 6;
    int    left_front_in2         = 5;
    int    left_rear_enb          = 13;
    int    left_rear_in3          = 20;
    int    left_rear_in4          = 16;

    int    right_front_ena        = 18;
    int    right_front_in1        = 27;
    int    right_front_in2        = 22;
    int    right_rear_enb         = 12;
    int    right_rear_in3         = 23;
    int    right_rear_in4         = 24;

    float  max_throttle           = 0.6f;   // normalised 0–1
    float  max_steering           = 1.0f;   // normalised −1 to 1
    int    watchdog_timeout_ms    = 200;    // zero PWM if no command within this

    int    spin_360_duration_ms   = 3000;
    int    turn_180_duration_ms   = 1500;
};
