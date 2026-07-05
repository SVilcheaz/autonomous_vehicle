#include "rc_receiver.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include <asm/termbits.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {
constexpr uint8_t kSyncByte           = 0xC8;  // CRSF_ADDRESS_FLIGHT_CONTROLLER
constexpr uint8_t kFrameTypeRcChannel = 0x16;  // CRSF_FRAMETYPE_RC_CHANNELS_PACKED
constexpr size_t  kPayloadLen         = 22;    // 16 channels * 11 bits
}

RCReceiver::RCReceiver(const PipelineConfig& cfg) : cfg_(cfg) {}

RCReceiver::~RCReceiver() { stop(); }

bool RCReceiver::init() {
    fd_ = open(cfg_.rc_serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        fprintf(stderr, "[RC] Failed to open %s\n", cfg_.rc_serial_port.c_str());
        return false;
    }

    // 420000 baud isn't representable by a standard termios speed_t constant,
    // so we set it via the Linux-specific termios2/BOTHER ioctl interface.
    struct termios2 tio {};
    if (ioctl(fd_, TCGETS2, &tio) != 0) {
        fprintf(stderr, "[RC] TCGETS2 failed on %s\n", cfg_.rc_serial_port.c_str());
        close(fd_);
        fd_ = -1;
        return false;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = cfg_.rc_baud;
    tio.c_ospeed = cfg_.rc_baud;

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8 | CREAD | CLOCAL;
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (ioctl(fd_, TCSETS2, &tio) != 0) {
        fprintf(stderr, "[RC] TCSETS2 failed on %s\n", cfg_.rc_serial_port.c_str());
        close(fd_);
        fd_ = -1;
        return false;
    }

    fprintf(stdout, "[RC] Opened %s @ %d baud\n", cfg_.rc_serial_port.c_str(), cfg_.rc_baud);
    return true;
}

void RCReceiver::start() {
    if (fd_ < 0) return;
    running_ = true;
    thread_ = std::thread(&RCReceiver::read_loop, this);
}

void RCReceiver::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void RCReceiver::read_loop() {
    fprintf(stdout, "[RC] Receiver started\n");

    std::vector<uint8_t> buf;
    uint8_t chunk[64];

    while (running_) {
        ssize_t n = read(fd_, chunk, sizeof(chunk));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        buf.insert(buf.end(), chunk, chunk + n);

        while (buf.size() >= 2) {
            // Not a sync byte: drop it and keep scanning, so a single
            // corrupted byte can't permanently wedge the parser.
            if (buf[0] != kSyncByte) {
                buf.erase(buf.begin());
                continue;
            }

            size_t length = buf[1];
            if (buf.size() < length + 2) break;  // wait for the rest of the frame

            uint8_t frame_type = buf[2];
            if (frame_type == kFrameTypeRcChannel && length >= 1 + kPayloadLen) {
                handle_frame(&buf[3]);
            }
            buf.erase(buf.begin(), buf.begin() + length + 2);
        }
    }

    fprintf(stdout, "[RC] Receiver stopped\n");
}

void RCReceiver::handle_frame(const uint8_t* payload) {
    std::array<int, kNumChannels> ch{};

    uint32_t accum      = 0;
    int      bits_ready = 0;
    size_t   byte_idx   = 0;
    for (int i = 0; i < kNumChannels; ++i) {
        while (bits_ready < 11) {
            accum |= static_cast<uint32_t>(payload[byte_idx++]) << bits_ready;
            bits_ready += 8;
        }
        ch[i] = static_cast<int>(accum & 0x7FF);
        accum >>= 11;
        bits_ready -= 11;
    }

    std::lock_guard<std::mutex> lock(channels_mtx_);
    channels_      = ch;
    last_frame_ms_ = now_ms();
}

float RCReceiver::normalize(int raw, bool invert) const {
    float norm;
    if (raw >= cfg_.rc_channel_mid) {
        norm = static_cast<float>(raw - cfg_.rc_channel_mid) /
               static_cast<float>(cfg_.rc_channel_max - cfg_.rc_channel_mid);
    } else {
        norm = static_cast<float>(raw - cfg_.rc_channel_mid) /
               static_cast<float>(cfg_.rc_channel_mid - cfg_.rc_channel_min);
    }
    norm = std::clamp(norm, -1.0f, 1.0f);
    if (std::abs(norm) < cfg_.rc_deadzone) norm = 0.0f;
    return invert ? -norm : norm;
}

bool RCReceiver::manual_switch_active() const {
    std::lock_guard<std::mutex> lock(channels_mtx_);
    return channels_[cfg_.rc_ch_mode_switch] > cfg_.rc_switch_threshold;
}

bool RCReceiver::signal_fresh(int timeout_ms) const {
    std::lock_guard<std::mutex> lock(channels_mtx_);
    if (last_frame_ms_ == 0) return false;
    return (now_ms() - last_frame_ms_) < timeout_ms;
}

DriveCommand RCReceiver::get_drive_command() const {
    int steer_raw, throttle_raw;
    {
        std::lock_guard<std::mutex> lock(channels_mtx_);
        steer_raw    = channels_[cfg_.rc_ch_steering];
        throttle_raw = channels_[cfg_.rc_ch_throttle];
    }

    DriveCommand cmd;
    cmd.steering = normalize(steer_raw, cfg_.rc_invert_steering);
    cmd.throttle = normalize(throttle_raw, cfg_.rc_invert_throttle);
    return cmd;
}
