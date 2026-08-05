#include "ti_vision/uart_publisher.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace ti_vision {
namespace {

constexpr float kMaxLinearExtrapolationS = 0.012F;

speed_t baud_constant(int baud) {
    if (baud == 115200) {
        return B115200;
    }
    throw std::invalid_argument("only 115200 baud is supported");
}

void write_all(int fd, const std::uint8_t* data, std::size_t length) {
    std::size_t offset = 0;
    while (offset < length) {
        const ssize_t written = write(fd, data + offset, length - offset);
        if (written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        throw std::runtime_error(std::string("UART write: ") + std::strerror(errno));
    }
}

}

UartPublisher::UartPublisher(std::string device,
                             int baud,
                             double rate_hz,
                             int stale_timeout_ms,
                             std::int8_t target_control)
    : device_(std::move(device)),
      baud_(baud),
      rate_hz_(rate_hz),
      target_control_(target_control),
      stale_timeout_(stale_timeout_ms) {
    if (rate_hz <= 0.0 || stale_timeout_ms <= 0) {
        throw std::invalid_argument("invalid UART publisher configuration");
    }
}

UartPublisher::~UartPublisher() {
    stop();
}

void UartPublisher::start() {
    if (running_.exchange(true)) {
        return;
    }
    try {
        open_port();
        thread_ = std::thread(&UartPublisher::run, this);
    } catch (...) {
        running_ = false;
        close_port();
        throw;
    }
}

void UartPublisher::stop() {
    if (!running_.exchange(false)) {
        close_port();
        return;
    }
    wake_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    close_port();
}

void UartPublisher::publish(float position_cm,
                            float speed_cm_s,
                            std::uint64_t) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_position_cm_ = position_cm;
        latest_speed_cm_s_ = speed_cm_s;
        latest_time_ = std::chrono::steady_clock::now();
        has_value_ = true;
    }
    wake_.notify_one();
}

void UartPublisher::invalidate() {
    std::lock_guard<std::mutex> lock(mutex_);
    has_value_ = false;
}

void UartPublisher::set_target_control(std::int8_t target_control) noexcept {
    target_control_.store(target_control, std::memory_order_relaxed);
}

std::int8_t UartPublisher::target_control() const noexcept {
    return target_control_.load(std::memory_order_relaxed);
}

std::uint64_t UartPublisher::sent_frames() const noexcept {
    return sent_frames_.load();
}

std::uint64_t UartPublisher::deadline_misses() const noexcept {
    return deadline_misses_.load();
}

std::array<std::uint8_t, 11> UartPublisher::encode(
    float position_cm,
    float speed_cm_s,
    std::int8_t target_control) {
    static_assert(sizeof(float) == 4, "protocol requires IEEE-754 float32");
    std::array<std::uint8_t, 11> packet{
        0xA5, 0x09, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::memcpy(packet.data() + 2, &position_cm, sizeof(position_cm));
    std::memcpy(packet.data() + 6, &speed_cm_s, sizeof(speed_cm_s));
    packet[10] = static_cast<std::uint8_t>(target_control);
    return packet;
}

void UartPublisher::run() {
    const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count > 1) {
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        CPU_SET(static_cast<int>(cpu_count - 1), &cpu_set);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
    }
    sched_param realtime{};
    realtime.sched_priority = sched_get_priority_min(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &realtime);

    const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1000000000.0 / rate_hz_));
    auto deadline = std::chrono::steady_clock::now() + period;

    while (running_.load()) {
        std::this_thread::sleep_until(deadline);
        if (!running_.load()) {
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        float position = 0.0F;
        float speed = 0.0F;
        std::chrono::steady_clock::time_point sample_time;
        bool fresh = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fresh = has_value_ && now - latest_time_ <= stale_timeout_;
            position = latest_position_cm_;
            speed = latest_speed_cm_s_;
            sample_time = latest_time_;
        }
        if (fresh) {
            const float age_s = std::chrono::duration<float>(now - sample_time).count();
            const float extrapolation_s = std::clamp(
                age_s,
                0.0F,
                kMaxLinearExtrapolationS);
            position += speed * extrapolation_s;
            const auto packet = encode(position, speed, target_control());
            write_all(fd_, packet.data(), packet.size());
            ++sent_frames_;
        }

        if (now > deadline + period) {
            ++deadline_misses_;
            deadline = now + period;
        } else {
            deadline += period;
        }
    }
}

void UartPublisher::open_port() {
    fd_ = open(device_.c_str(), O_WRONLY | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
        throw std::runtime_error("open " + device_ + ": " + std::strerror(errno));
    }

    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
        close_port();
        throw std::runtime_error(std::string("tcgetattr: ") + std::strerror(errno));
    }
    cfmakeraw(&options);
    const speed_t speed = baud_constant(baud_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        close_port();
        throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(errno));
    }
    tcflush(fd_, TCOFLUSH);
}

void UartPublisher::close_port() noexcept {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

}
