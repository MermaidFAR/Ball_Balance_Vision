#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ti_vision/recording_store.hpp"
#include "ti_vision/types.hpp"

namespace ti_vision {

enum class TxMode : int {
    Tracking = 0,
    Manual = 1,
    Stopped = 2,
};

class WebServer {
public:
    WebServer(
        int port,
        int preview_fps,
        std::string recordings_directory,
        float zero_px,
        float cm_per_px,
        std::int8_t initial_target_control);
    ~WebServer();

    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    void start();
    void stop();

    TxMode tx_mode() const noexcept;
    std::int8_t target_control() const noexcept;
    float manual_cm() const noexcept;

    void update(
        const std::uint8_t* y,
        std::size_t y_size,
        int width,
        int height,
        const Detection& detection,
        float measured_cm,
        float raw_speed_cm_s,
        float measured_speed_cm_s,
        float sent_cm,
        float sent_speed_cm_s,
        bool transmitting,
        double decode_fps,
        double infer_fps,
        std::uint64_t decoded_frames,
        std::uint64_t inferred_frames,
        std::uint64_t dropped_frames,
        std::uint64_t decoder_resets,
        std::uint64_t tx_frames,
        std::uint64_t tx_deadline_misses);

private:
    struct Snapshot {
        std::vector<std::uint8_t> y;
        int width = 0;
        int height = 0;
        Detection detection;
        float measured_cm = 0.0F;
        float raw_speed_cm_s = 0.0F;
        float measured_speed_cm_s = 0.0F;
        float sent_cm = 0.0F;
        float sent_speed_cm_s = 0.0F;
        bool transmitting = false;
        double decode_fps = 0.0;
        double infer_fps = 0.0;
        std::uint64_t decoded_frames = 0;
        std::uint64_t inferred_frames = 0;
        std::uint64_t dropped_frames = 0;
        std::uint64_t decoder_resets = 0;
        std::uint64_t tx_frames = 0;
        std::uint64_t tx_deadline_misses = 0;
        std::uint64_t updated_ns = 0;
    };

    void run();
    void handle_client(int client_fd);
    std::string status_json();
    std::string recordings_json();
    bool apply_control(const std::string& body, std::string& error);
    bool apply_recording(const std::string& body, std::string& response, std::string& error);
    bool send_replay_frame(int client_fd, const std::string& path);

    int port_ = 0;
    int preview_fps_ = 0;
    float zero_px_ = 0.0F;
    float cm_per_px_ = 0.0F;
    int listen_fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<int> tx_mode_{static_cast<int>(TxMode::Tracking)};
    std::atomic<std::int8_t> target_control_{0};
    std::atomic<float> manual_cm_{0.0F};
    std::mutex snapshot_mutex_;
    Snapshot snapshot_;
    std::uint64_t next_preview_ns_ = 0;
    RecordingStore recordings_;
};

}
