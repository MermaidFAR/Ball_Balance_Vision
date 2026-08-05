#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace ti_vision {

class UartPublisher {
public:
    UartPublisher(std::string device,
                  int baud,
                  double rate_hz,
                  int stale_timeout_ms,
                  std::int8_t target_control);
    ~UartPublisher();

    UartPublisher(const UartPublisher&) = delete;
    UartPublisher& operator=(const UartPublisher&) = delete;

    void start();
    void stop();
    void publish(float position_cm,
                 float speed_cm_s,
                 std::uint64_t detected_ns);
    void invalidate();
    void set_target_control(std::int8_t target_control) noexcept;
    std::int8_t target_control() const noexcept;
    std::uint64_t sent_frames() const noexcept;
    std::uint64_t deadline_misses() const noexcept;

    static std::array<std::uint8_t, 11> encode(float position_cm,
                                               float speed_cm_s,
                                               std::int8_t target_control);

private:
    void run();
    void open_port();
    void close_port() noexcept;

    std::string device_;
    int baud_ = 0;
    double rate_hz_ = 0.0;
    std::atomic<std::int8_t> target_control_{1};
    std::chrono::milliseconds stale_timeout_{0};
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    float latest_position_cm_ = 0.0F;
    float latest_speed_cm_s_ = 0.0F;
    std::chrono::steady_clock::time_point latest_time_{};
    bool has_value_ = false;
    std::atomic<std::uint64_t> sent_frames_{0};
    std::atomic<std::uint64_t> deadline_misses_{0};
};

}
