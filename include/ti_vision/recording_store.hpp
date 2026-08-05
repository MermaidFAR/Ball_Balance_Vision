#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ti_vision/types.hpp"

namespace ti_vision {

struct RecordingInfo {
    std::string id;
    std::uint64_t started_unix_ms = 0;
    std::uint64_t frame_count = 0;
    std::uint64_t duration_ms = 0;
    int width = 0;
    int height = 0;
    int fps = 0;
    std::uint64_t bytes = 0;
};

struct RecordedFrame {
    std::vector<std::uint8_t> y;
    std::uint64_t elapsed_ms = 0;
    float measured_cm = 0.0F;
    float sent_cm = 0.0F;
    std::int8_t target_control = 0;
    Detection detection;
};

class RecordingStore {
public:
    RecordingStore(std::string directory, int width, int height, int fps);
    ~RecordingStore();

    RecordingStore(const RecordingStore&) = delete;
    RecordingStore& operator=(const RecordingStore&) = delete;

    void start_worker();
    void shutdown();

    bool start_recording(std::string& id, std::string& error);
    void stop_recording();
    void enqueue(
        const std::uint8_t* y,
        std::size_t y_size,
        float measured_cm,
        float sent_cm,
        std::int8_t target_control,
        const Detection& detection);

    bool recording() const noexcept;
    std::string active_id() const;
    std::uint64_t dropped_frames() const noexcept;
    std::vector<RecordingInfo> list() const;
    bool read_frame(
        const std::string& id,
        std::uint64_t index,
        RecordedFrame& frame,
        std::string& error) const;

private:
    enum class ItemType {
        Start,
        Frame,
        Stop,
    };

    struct QueueItem {
        ItemType type = ItemType::Frame;
        std::string id;
        std::uint64_t started_unix_ms = 0;
        RecordedFrame frame;
    };

    void writer_loop();
    bool valid_id(const std::string& id) const;

    std::string directory_;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;
    std::atomic<bool> worker_running_{false};
    std::atomic<bool> recording_{false};
    std::atomic<std::uint64_t> dropped_frames_{0};
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<QueueItem> queue_;
    std::thread writer_thread_;
    std::string active_id_;
    std::uint64_t recording_started_steady_ms_ = 0;
};

}
