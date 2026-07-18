#include <cstdio>
#include <chrono>
#include <thread>
#include "rc_receiver.hpp"
#include "config.hpp"
#include <iostream>

int main() {
    const PipelineConfig cfg;
    RCReceiver rc(cfg);

    if (!rc.init()) {
        fprintf(stderr, "[rc_receiver_test] Failed to open %s\n", cfg.rc_serial_port.c_str());
        return 1;
    }
    rc.start();

    fprintf(stdout, "[rc_receiver_test] Reading channels — wiggle sticks/switches to confirm mapping.\n"
                     "Expected: ch%d=steering, ch%d=throttle, ch%d=mode switch (low<%d=AUTOPILOT, "
                     "mid=FOLLOW, high>%d=MANUAL)\n",
            cfg.rc_ch_steering, cfg.rc_ch_throttle, cfg.rc_ch_mode_switch,
            cfg.rc_switch_threshold_low, cfg.rc_switch_threshold_high);

    const char* mode_names[] = {"IDLE", "FOLLOW", "AUTOPILOT", "MANUAL"};
    DriveMode mode = DriveMode::IDLE;
    while (true) {
        
        bool fresh = rc.signal_fresh(cfg.rc_signal_timeout_ms);
        if (rc.activate_switch_reading()){
            mode = rc.selected_drive_mode();
        }
        DriveCommand cmd = rc.get_drive_command();

        fprintf(stdout, "[rc_receiver_test] fresh=%d mode=%s throttle=%+.2f steering=%+.2f\n",
                fresh, mode_names[(int)mode], cmd.throttle, cmd.steering);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
