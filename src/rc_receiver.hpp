#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>

#include "types.hpp"
#include "config.hpp"

// Reads a CRSF-protocol ELRS receiver (see test_rc.py for the raw wire
// format this mirrors) and exposes the latest channel snapshot as
// continuously-polled state, not a queued stream — DecisionEngine reads
// "what are the sticks doing right now", never a backlog of past frames.
class RCReceiver {
public:
    static constexpr int kNumChannels = 16;

    explicit RCReceiver(const PipelineConfig& cfg);
    ~RCReceiver();

    bool init();
    void start();
    void stop();

    // Buckets the mode-switch channel into AUTOPILOT (low) / FOLLOW (mid) /
    // MANUAL (high) per the configured thresholds. Only meaningful when
    // signal_fresh() is true — it reads whatever the last received frame said.
    DriveMode selected_drive_mode() const;
    bool signal_fresh(int timeout_ms) const;
    DriveCommand get_drive_command() const;
    bool activate_switch_reading() const;

private:
    void read_loop();
    void handle_frame(const uint8_t* payload);
    float normalize(int raw, bool invert) const;

    const PipelineConfig& cfg_;

    int fd_{-1};
    std::atomic<bool> running_{false};
    std::thread thread_;

    mutable std::mutex channels_mtx_;
    std::array<int, kNumChannels> channels_{};
    int64_t last_frame_ms_{0};
};
