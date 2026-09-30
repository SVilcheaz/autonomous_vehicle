#include "session_logger.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>

namespace {
bool write_all(int fd, const std::string& line) {
    const char* data = line.data();
    size_t remaining = line.size();
    while (remaining > 0) {
        ssize_t n = write(fd, data, remaining);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        data += n;
        remaining -= static_cast<size_t>(n);
    }
    return true;
}

std::string timestamped(const std::string& line) {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    auto seconds = std::chrono::system_clock::to_time_t(now);
    struct tm local {};
    localtime_r(&seconds, &local);
    char prefix[48];
    snprintf(prefix, sizeof(prefix), "%04d-%02d-%02d %02d:%02d:%02d.%03d ",
             local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
             local.tm_hour, local.tm_min, local.tm_sec, (int)ms);
    return std::string(prefix) + line + '\n';
}
}

SessionLogger::~SessionLogger() { stop(); }

bool SessionLogger::start(const std::filesystem::path& path) {
    log_fd_ = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log_fd_ < 0) {
        perror("[Record] Cannot open pipeline log");
        return false;
    }
    console_fd_ = dup(STDOUT_FILENO);
    original_stderr_fd_ = dup(STDERR_FILENO);
    int pipe_fds[2];
    if (console_fd_ < 0 || original_stderr_fd_ < 0 || pipe(pipe_fds) != 0) {
        perror("[Record] Cannot create log pipe");
        stop();
        return false;
    }

    fflush(stdout);
    fflush(stderr);
    if (dup2(pipe_fds[1], STDOUT_FILENO) < 0 ||
        dup2(pipe_fds[1], STDERR_FILENO) < 0) {
        perror("[Record] Cannot redirect logs");
        if (console_fd_ >= 0) dup2(console_fd_, STDOUT_FILENO);
        if (original_stderr_fd_ >= 0) dup2(original_stderr_fd_, STDERR_FILENO);
        close(pipe_fds[0]);
        close(pipe_fds[1]);
        stop();
        return false;
    }
    close(pipe_fds[1]);
    setvbuf(stdout, nullptr, _IOLBF, 0);
    active_ = true;
    thread_ = std::thread(&SessionLogger::read_loop, this, pipe_fds[0]);
    return true;
}

void SessionLogger::stop() {
    if (active_) {
        fflush(stdout);
        fflush(stderr);
        dup2(console_fd_, STDOUT_FILENO);
        dup2(original_stderr_fd_, STDERR_FILENO);
        active_ = false;
    }
    if (thread_.joinable()) thread_.join();
    if (console_fd_ >= 0) close(console_fd_);
    if (original_stderr_fd_ >= 0) close(original_stderr_fd_);
    if (log_fd_ >= 0) close(log_fd_);
    console_fd_ = original_stderr_fd_ = log_fd_ = -1;
}

void SessionLogger::read_loop(int read_fd) {
    char buffer[4096];
    std::string pending;
    while (true) {
        ssize_t n = read(read_fd, buffer, sizeof(buffer));
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) break;
        pending.append(buffer, static_cast<size_t>(n));
        size_t end;
        while ((end = pending.find('\n')) != std::string::npos) {
            std::string line = timestamped(pending.substr(0, end));
            if (!write_all(log_fd_, line)) failed_ = true;
            write_all(console_fd_, line);
            pending.erase(0, end + 1);
        }
    }
    if (!pending.empty()) {
        std::string line = timestamped(pending);
        if (!write_all(log_fd_, line)) failed_ = true;
        write_all(console_fd_, line);
    }
    close(read_fd);
}
