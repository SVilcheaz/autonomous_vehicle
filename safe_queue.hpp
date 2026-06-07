#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstddef>

// Ring buffer with blocking pop.
// push() always succeeds: if the buffer is full it evicts the oldest entry
// so the consumer always sees the most recent data, never stale frames.
template<typename T>
class SafeQueue {
public:
    explicit SafeQueue(size_t max_size = 2)
        : max_size_(max_size) {}

    void push(T val) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (q_.size() >= max_size_) {
            q_.pop_front();   // evict oldest
        }
        q_.push_back(std::move(val));
        cv_.notify_one();
    }

    // Block until an item is available or timeout_ms elapses.
    // Returns false on timeout.
    bool pop(T& val, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mtx_);
        bool available = cv_.wait_for(lock,
            std::chrono::milliseconds(timeout_ms),
            [this] { return !q_.empty(); });
        if (!available) return false;
        val = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return q_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return q_.empty();
    }

private:
    std::deque<T>           q_;
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    size_t                  max_size_;
};
