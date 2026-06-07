#pragma once

#include <cstddef>

struct PipelineConfig {
    // ── Camera (T1) ───────────────────────────────────────────────────────────
    int    camera_width        = 1280;
    int    camera_height       = 720;
    size_t frame_queue_size    = 2;    // ring buffer slots between T1 and T2

    // ── Inference (T2) ────────────────────────────────────────────────────────
    float  conf_threshold      = 0.4f;
    float  nms_iou_threshold   = 0.45f;
    size_t perception_queue_size = 2;  // ring buffer slots between T2 and T3

    // ── Decision (T3) ────────────────────────────────────────────────────────
    float  obstacle_dist_m     = 1.0f; // repulsion kicks in below this depth (metres)
    size_t command_queue_size  = 2;    // ring buffer slots between T3 and T4

    // ── Actuation (T4) ───────────────────────────────────────────────────────
    float  max_throttle        = 0.6f; // normalised 0–1
    float  max_steering        = 1.0f; // normalised −1 to 1
    int    watchdog_timeout_ms = 200;  // zero PWM if no command arrives within this
};
