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
    float  pi_kp                  = 0.0;   // throttle: distance-hold gain
    float  pi_ki                  = 0.0;

    float  pd_kp                  = 0.0;   // steering: person-centering gain
    float  pd_kd                  = 0.0;

    // Target distance to the tracked person in FOLLOW mode, in the same
    // raw/uncalibrated depth-model units as Detection::depth (see
    // perception_result.hpp) and obstacle_dist_m above — read the printed
    // depth at the desired stand-off distance during bench testing and set
    // this to match.
    float  follow_target_depth    = 0.5f;
    // Flip if the depth model turns out to report inverse depth (larger
    // value = closer) rather than the assumed larger = farther.
    bool   follow_invert_depth    = false;

    // ── RC Receiver (Manual override) ───────────────────────────────────────
    std::string rc_serial_port    = "/dev/serial0";
    int    rc_baud                = 420000;
    int    rc_ch_steering         = 3;      // CRSF channel index (AETR: Aileron)
    int    rc_ch_throttle         = 1;      // CRSF channel index (AETR: Throttle)
    int    rc_ch_mode_switch      = 6;      // CRSF channel index (AUX1)
    int    rc_switch_threshold    = 1500;   // raw value above => MANUAL active
    int    rc_channel_min         = 172;
    int    rc_channel_mid         = 992;
    int    rc_channel_max         = 1811;
    float  rc_deadzone            = 0.05f;  // normalised, applied around center
    bool   rc_invert_steering     = false;
    bool   rc_invert_throttle     = false;
    int    rc_signal_timeout_ms   = 150;    // no valid frame within this => failsafe

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
