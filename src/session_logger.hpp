#pragma once

#include <atomic>
#include <filesystem>
#include <thread>

// Mirrors all stdout and stderr (including worker threads) to the console
// and a timestamped log file for a recorded run.
class SessionLogger {
public:
    SessionLogger() = default;
    ~SessionLogger();

    bool start(const std::filesystem::path& path);
    void stop();
    bool failed() const { return failed_.load(); }

private:
    void read_loop(int read_fd);

    int log_fd_{-1};
    int console_fd_{-1};
    int original_stderr_fd_{-1};
    bool active_{false};
    std::atomic<bool> failed_{false};
    std::thread thread_;
};
