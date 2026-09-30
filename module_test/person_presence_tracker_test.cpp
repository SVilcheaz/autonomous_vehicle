#include <cassert>
#include <cstdio>

#include "person_presence_tracker.hpp"

int main() {
    PersonPresenceTracker tracker;
    PerceptionResult empty;
    assert(tracker.update(empty).kind == PersonEvent::Kind::NONE);

    PerceptionResult person;
    person.detections.push_back({"person", 0.92f, 10, 20, 110, 220, 0.6f});
    auto event = tracker.update(person);
    assert(event.kind == PersonEvent::Kind::DETECTED);
    assert(event.count == 1);
    assert(event.target.depth == 0.6f);
    assert(tracker.update(person).kind == PersonEvent::Kind::NONE);

    // One missed detection does not count as losing the person.
    assert(tracker.update(empty).kind == PersonEvent::Kind::NONE);
    assert(tracker.update(person).kind == PersonEvent::Kind::NONE);
    assert(tracker.update(empty).kind == PersonEvent::Kind::NONE);

    PerceptionResult car;
    car.detections.push_back({"car", 0.95f, 0, 0, 100, 100, 0.4f});
    assert(tracker.update(car).kind == PersonEvent::Kind::LOST);
    assert(tracker.update(car).kind == PersonEvent::Kind::NONE);
    assert(tracker.update(person).kind == PersonEvent::Kind::DETECTED);

    fprintf(stdout, "[person_event_test] arrival, missed frame, loss, reacquisition verified\n");
    return 0;
}
