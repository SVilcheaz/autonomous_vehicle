#pragma once

#include "perception_result.hpp"

struct PersonEvent {
    enum class Kind { NONE, DETECTED, LOST } kind{Kind::NONE};
    int count{0};
    Detection target{};
};

// Reports arrival once, then waits for two empty inference results before
// reporting loss. This prevents one missed YOLO detection from spamming logs.
class PersonPresenceTracker {
public:
    PersonEvent update(const PerceptionResult& result) {
        PersonEvent event;
        int largest_area = -1;
        for (const auto& det : result.detections) {
            if (det.label != "person") continue;
            ++event.count;
            int area = (det.x2 - det.x1) * (det.y2 - det.y1);
            if (area > largest_area) {
                largest_area = area;
                event.target = det;
            }
        }

        if (event.count > 0) {
            missing_frames_ = 0;
            if (!visible_) {
                visible_ = true;
                event.kind = PersonEvent::Kind::DETECTED;
            }
        } else if (visible_ && ++missing_frames_ >= 2) {
            visible_ = false;
            missing_frames_ = 0;
            event.kind = PersonEvent::Kind::LOST;
        }
        return event;
    }

private:
    bool visible_{false};
    int missing_frames_{0};
};
