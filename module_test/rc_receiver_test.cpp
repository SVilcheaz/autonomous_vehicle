#include <cstdio>
#include <chrono>
#include <thread>
#include "rc_receiver.hpp"
#include "config.hpp"

int main() {
    const PipelineConfig cfg;
    RCReceiver rc(cfg);

    if (!rc.init()) {
        fprintf(stderr, "[rc_receiver_test] Failed to open %s\n", cfg.rc_serial_port.c_str());
        return 1;
    }
    rc.start();

    fprintf(stdout, "[rc_receiver_test] Reading channels — wiggle sticks/switches to confirm mapping.\n"
                     "Expected: ch%d=steering, ch%d=throttle, ch%d=manual switch (threshold %d)\n",
            cfg.rc_ch_steering, cfg.rc_ch_throttle, cfg.rc_ch_mode_switch, cfg.rc_switch_threshold);

    while (true) {
        bool fresh  = rc.signal_fresh(cfg.rc_signal_timeout_ms);
        bool manual = rc.manual_switch_active();
        DriveCommand cmd = rc.get_drive_command();

        fprintf(stdout, "[rc_receiver_test] fresh=%d manual=%d throttle=%+.2f steering=%+.2f\n",
                fresh, manual, cmd.throttle, cmd.steering);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
