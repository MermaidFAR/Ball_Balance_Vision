#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ti_vision/bpu_detector.hpp"
#include "ti_vision/mjpeg_decoder.hpp"
#include "ti_vision/roi_extractor.hpp"
#include "ti_vision/uart_publisher.hpp"
#include "ti_vision/v4l2_camera.hpp"
#include "ti_vision/web_server.hpp"

namespace {

constexpr int kCameraWidth = 640;
constexpr int kCameraHeight = 480;
constexpr int kCameraFps = 120;
constexpr int kCropY = 192;
constexpr int kCropHeight = 96;
constexpr float kLostLimitCm = 10.0F;
constexpr float kSpeedObserverTauS = 0.040F;

std::atomic<bool> running{true};

void signal_handler(int) {
    running = false;
}

struct Calibration {
    float zero_px = 333.0925F;
    float cm_per_px = 0.02201645F;
};

struct Options {
    std::string camera = "/dev/video0";
    std::string model = "/root/TI/models/compiled/ball_yolo26n_640x96_nv12.bin";
    std::string uart;
    int camera_buffers = 4;
    int exposure_100us = 50;
    int duration_seconds = 0;
    int stale_timeout_ms = 80;
    int target_control = 0;
    float threshold = 0.20F;
    float lost_ramp_cm_s = 2.0F;
    double send_rate_hz = 500.0;
    int web_port = 8080;
    int preview_fps = 8;
    std::string recordings_directory = "/root/TI/recordings";
    Calibration calibration;
};

struct RoiMailbox {
    std::vector<std::uint8_t> y;
    std::vector<std::uint8_t> uv;
    std::uint64_t sequence = 0;
    std::uint64_t captured_ns = 0;
    std::uint64_t generation = 0;
};

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --camera PATH             default /dev/video0\n"
        << "  --model PATH              compiled 640x96 NV12 model\n"
        << "  --duration SEC            0 means run until SIGINT\n"
        << "  --threshold SCORE         default 0.20\n"
        << "  --exposure-100us N        default 50 (5 ms)\n"
        << "  --zero-px X               calibrated zero pixel\n"
        << "  --cm-per-px X             calibrated scale\n"
        << "  --lost-ramp-cm-s X        lost-ball ramp rate, default 2 cm/s\n"
        << "  --uart PATH               enable 500 Hz UART publishing\n"
        << "  --send-rate HZ            default 500\n"
        << "  --stale-timeout-ms MS     stop TX if the vision loop stalls\n"
        << "  --target-control N        0=target 0, 1=third-task ramp, others=target cm\n"
        << "  --web-port PORT           control page port, 0 disables, default 8080\n"
        << "  --preview-fps FPS         preview and recording rate, default 8\n"
        << "  --recordings-dir PATH     default /root/TI/recordings\n";
}

template <typename T>
T parse_number(const char* text, const char* name);

template <>
int parse_number<int>(const char* text, const char* name) {
    try {
        std::size_t used = 0;
        const int value = std::stoi(text, &used);
        if (used != std::string(text).size()) {
            throw std::invalid_argument("trailing data");
        }
        return value;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("invalid ") + name + ": " + text);
    }
}

template <>
float parse_number<float>(const char* text, const char* name) {
    try {
        std::size_t used = 0;
        const float value = std::stof(text, &used);
        if (used != std::string(text).size() || !std::isfinite(value)) {
            throw std::invalid_argument("invalid float");
        }
        return value;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("invalid ") + name + ": " + text);
    }
}

template <>
double parse_number<double>(const char* text, const char* name) {
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != std::string(text).size() || !std::isfinite(value)) {
            throw std::invalid_argument("invalid double");
        }
        return value;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("invalid ") + name + ": " + text);
    }
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (index + 1 >= argc) {
            throw std::invalid_argument("missing value for " + argument);
        }
        const char* value = argv[++index];
        if (argument == "--camera") options.camera = value;
        else if (argument == "--model") options.model = value;
        else if (argument == "--uart") options.uart = value;
        else if (argument == "--duration") options.duration_seconds = parse_number<int>(value, "duration");
        else if (argument == "--threshold") options.threshold = parse_number<float>(value, "threshold");
        else if (argument == "--exposure-100us") options.exposure_100us = parse_number<int>(value, "exposure");
        else if (argument == "--zero-px") options.calibration.zero_px = parse_number<float>(value, "zero-px");
        else if (argument == "--cm-per-px") options.calibration.cm_per_px = parse_number<float>(value, "cm-per-px");
        else if (argument == "--lost-ramp-cm-s") options.lost_ramp_cm_s = parse_number<float>(value, "lost-ramp-cm-s");
        else if (argument == "--send-rate") options.send_rate_hz = parse_number<double>(value, "send-rate");
        else if (argument == "--stale-timeout-ms") options.stale_timeout_ms = parse_number<int>(value, "stale-timeout-ms");
        else if (argument == "--target-control") options.target_control = parse_number<int>(value, "target-control");
        else if (argument == "--web-port") options.web_port = parse_number<int>(value, "web-port");
        else if (argument == "--preview-fps") options.preview_fps = parse_number<int>(value, "preview-fps");
        else if (argument == "--recordings-dir") options.recordings_directory = value;
        else throw std::invalid_argument("unknown option: " + argument);
    }

    if (options.duration_seconds < 0 || options.exposure_100us < 0 ||
        options.stale_timeout_ms <= 0 || options.threshold < 0.0F ||
        options.threshold > 1.0F || options.calibration.cm_per_px <= 0.0F ||
        options.lost_ramp_cm_s <= 0.0F || options.send_rate_hz <= 0.0 ||
        options.target_control < -128 || options.target_control > 127 ||
        options.web_port < 0 || options.web_port > 65535 ||
        options.preview_fps <= 0 || options.preview_fps > 30 ||
        options.recordings_directory.empty()) {
        throw std::invalid_argument("invalid timing, threshold, or calibration option");
    }
    return options;
}

float pixel_to_cm(float x, const Calibration& calibration) {
    return std::clamp(
        (calibration.zero_px - x) * calibration.cm_per_px,
        -12.0F,
        12.0F);
}

float ramp_lost_position(float position_cm, int direction, float step_cm) {
    const float target = direction > 0 ? kLostLimitCm : -kLostLimitCm;
    if (direction > 0) {
        return std::min(target, position_cm + step_cm);
    }
    return std::max(target, position_cm - step_cm);
}

struct SpeedEstimate {
    float observed_position_cm = 0.0F;
    float raw_cm_s = 0.0F;
    float observed_cm_s = 0.0F;
};

class AlphaBetaSpeedEstimator {
public:
    void reset(float position_cm, std::uint64_t timestamp_ns) {
        initialized_ = true;
        position_cm_ = position_cm;
        measured_position_cm_ = position_cm;
        timestamp_ns_ = timestamp_ns;
        speed_cm_s_ = 0.0F;
    }

    SpeedEstimate update(float measured_position_cm, std::uint64_t timestamp_ns) {
        if (!initialized_ ||
            timestamp_ns <= timestamp_ns_ ||
            timestamp_ns - timestamp_ns_ > 100000000ULL) {
            reset(measured_position_cm, timestamp_ns);
            return {position_cm_, 0.0F, 0.0F};
        }

        const float dt_s = static_cast<float>(timestamp_ns - timestamp_ns_) * 1.0e-9F;
        const float raw_speed_cm_s =
            (measured_position_cm - measured_position_cm_) / dt_s;
        const float predicted_position_cm = position_cm_ + speed_cm_s_ * dt_s;
        const float residual_cm = measured_position_cm - predicted_position_cm;

        // Fixed critical-damping gains keep the estimator linear across the full
        // speed range. No dead zone, gain switching, outlier rejection, or clipping.
        const float lambda = std::exp(-dt_s / kSpeedObserverTauS);
        const float alpha = 1.0F - lambda * lambda;
        const float beta = (1.0F - lambda) * (1.0F - lambda);
        position_cm_ = predicted_position_cm + alpha * residual_cm;
        speed_cm_s_ += (beta / dt_s) * residual_cm;

        measured_position_cm_ = measured_position_cm;
        timestamp_ns_ = timestamp_ns;
        return {position_cm_, raw_speed_cm_s, speed_cm_s_};
    }

private:
    bool initialized_ = false;
    float position_cm_ = 0.0F;
    float measured_position_cm_ = 0.0F;
    float speed_cm_s_ = 0.0F;
    std::uint64_t timestamp_ns_ = 0;
};

}

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        ti_vision::BpuDetector detector(options.model, options.threshold);
        if (detector.input_width() != kCameraWidth || detector.input_height() != kCropHeight) {
            throw std::runtime_error("pipeline requires a 640x96 model");
        }

        std::unique_ptr<ti_vision::UartPublisher> publisher;
        if (!options.uart.empty()) {
            publisher = std::make_unique<ti_vision::UartPublisher>(
                options.uart,
                115200,
                options.send_rate_hz,
                options.stale_timeout_ms,
                static_cast<std::int8_t>(options.target_control));
            publisher->start();
        }

        std::unique_ptr<ti_vision::WebServer> web_server;
        if (options.web_port > 0) {
            web_server = std::make_unique<ti_vision::WebServer>(
                options.web_port,
                options.preview_fps,
                options.recordings_directory,
                options.calibration.zero_px,
                options.calibration.cm_per_px,
                static_cast<std::int8_t>(options.target_control));
            web_server->start();
        }

        const auto started = std::chrono::steady_clock::now();
        auto report_at = started + std::chrono::seconds(1);
        std::atomic<std::uint64_t> decoded_frames{0};
        std::atomic<std::uint64_t> decoder_resets{0};
        std::atomic<std::uint64_t> kernel_drops{0};
        std::uint64_t inferred_frames = 0;
        std::uint64_t detected_frames = 0;
        ti_vision::Detection last_detection;
        float measured_position_cm = 0.0F;
        float output_position_cm = 0.0F;
        float raw_speed_cm_s = 0.0F;
        float output_speed_cm_s = 0.0F;
        AlphaBetaSpeedEstimator speed_estimator;
        int last_direction = 0;
        bool lost_active = false;
        auto lost_updated_at = started;
        float sent_position_cm = 0.0F;
        float sent_speed_cm_s = 0.0F;
        ti_vision::TxMode last_tx_mode = web_server
                                             ? web_server->tx_mode()
                                             : ti_vision::TxMode::Tracking;

        RoiMailbox mailbox;
        mailbox.y.resize(static_cast<std::size_t>(detector.input_width() * detector.input_height()));
        mailbox.uv.resize(mailbox.y.size() / 2U);
        std::mutex mailbox_mutex;
        std::condition_variable mailbox_ready;
        std::exception_ptr capture_error;

        std::thread capture_thread([&] {
            try {
                ti_vision::V4L2Camera camera(
                    options.camera,
                    kCameraWidth,
                    kCameraHeight,
                    kCameraFps,
                    options.camera_buffers);
                camera.set_exposure_100us(options.exposure_100us);
                auto decoder = std::make_unique<ti_vision::MjpegDecoder>(
                    kCameraWidth, kCameraHeight);

                while (running.load()) {
                    const auto frame = camera.capture_latest(1000);
                    if (frame.drained_frames > 1) {
                        kernel_drops.fetch_add(frame.drained_frames - 1);
                    }
                    try {
                        const auto decoded = decoder->decode(frame.jpeg, frame.sequence);
                        {
                            std::lock_guard<std::mutex> lock(mailbox_mutex);
                            ti_vision::extract_nv12_roi(
                                decoded,
                                0,
                                kCropY,
                                kCameraWidth,
                                kCropHeight,
                                detector.input_width(),
                                detector.input_height(),
                                mailbox.y,
                                mailbox.uv);
                            mailbox.sequence = frame.sequence;
                            mailbox.captured_ns = frame.captured_ns;
                            ++mailbox.generation;
                        }
                        decoder->release_output();
                    } catch (const std::exception& error) {
                        ++decoder_resets;
                        std::cerr << "decoder error: " << error.what()
                                  << ", rebuilding\n";
                        decoder.reset();
                        while (running.load() && !decoder) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(20));
                            try {
                                decoder = std::make_unique<ti_vision::MjpegDecoder>(
                                    kCameraWidth, kCameraHeight);
                            } catch (const std::exception& restart_error) {
                                std::cerr << "decoder rebuild failed: "
                                          << restart_error.what() << '\n';
                            }
                        }
                        continue;
                    }
                    ++decoded_frames;
                    mailbox_ready.notify_one();
                }
            } catch (...) {
                {
                    std::lock_guard<std::mutex> lock(mailbox_mutex);
                    capture_error = std::current_exception();
                }
                running = false;
                mailbox_ready.notify_one();
            }
        });

        std::vector<std::uint8_t> local_y(mailbox.y.size());
        std::vector<std::uint8_t> local_uv(mailbox.uv.size());
        std::uint64_t consumed_generation = 0;

        std::cout << "vision started: 640x480@120 crop 640x96, lost ramp="
                  << options.lost_ramp_cm_s << "cm/s to +/-" << kLostLimitCm
                  << "cm, uart=" << (publisher ? options.uart : "disabled")
                  << ", target_control=" << options.target_control
                  << ", web="
                  << (web_server ? "0.0.0.0:" + std::to_string(options.web_port) : "disabled")
                  << ", recording=" << options.preview_fps << "fps raw grayscale\n";

        try {
            while (running.load()) {
                const auto now = std::chrono::steady_clock::now();
                if (options.duration_seconds > 0 &&
                    now - started >= std::chrono::seconds(options.duration_seconds)) {
                    break;
                }

                std::uint64_t sequence = 0;
                std::uint64_t captured_ns = 0;
                {
                    std::unique_lock<std::mutex> lock(mailbox_mutex);
                    mailbox_ready.wait_for(lock, std::chrono::milliseconds(100), [&] {
                        return mailbox.generation != consumed_generation || capture_error || !running.load();
                    });
                    if (capture_error || !running.load()) {
                        break;
                    }
                    if (mailbox.generation == consumed_generation) {
                        continue;
                    }
                    std::memcpy(local_y.data(), mailbox.y.data(), local_y.size());
                    std::memcpy(local_uv.data(), mailbox.uv.data(), local_uv.size());
                    sequence = mailbox.sequence;
                    captured_ns = mailbox.captured_ns;
                    consumed_generation = mailbox.generation;
                }

                const ti_vision::DecodedFrameView roi{
                    local_y.data(),
                    local_uv.data(),
                    detector.input_width(),
                    detector.input_height(),
                    detector.input_width()};
                auto result = detector.infer(roi, 0, sequence, captured_ns);
                if (result.score < options.threshold) {
                    result.valid = false;
                }

                ++inferred_frames;
                last_detection = result;
                const auto detected_at = std::chrono::steady_clock::now();

                if (result.valid) {
                    measured_position_cm = pixel_to_cm(result.cx, options.calibration);
                    if (lost_active) {
                        speed_estimator.reset(measured_position_cm, result.captured_ns);
                        output_position_cm = measured_position_cm;
                        raw_speed_cm_s = 0.0F;
                        output_speed_cm_s = 0.0F;
                    } else {
                        const SpeedEstimate speed = speed_estimator.update(
                            measured_position_cm,
                            result.captured_ns);
                        output_position_cm = speed.observed_position_cm;
                        raw_speed_cm_s = speed.raw_cm_s;
                        output_speed_cm_s = speed.observed_cm_s;
                    }
                    if (measured_position_cm > 0.0F) {
                        last_direction = 1;
                    } else if (measured_position_cm < 0.0F) {
                        last_direction = -1;
                    }
                    lost_active = false;
                    ++detected_frames;
                } else if (last_direction != 0) {
                    if (!lost_active) {
                        lost_active = true;
                        lost_updated_at = detected_at;
                    }
                    const float elapsed_s =
                        std::chrono::duration<float>(detected_at - lost_updated_at).count();
                    output_position_cm = ramp_lost_position(
                        output_position_cm,
                        last_direction,
                        options.lost_ramp_cm_s * elapsed_s);
                    raw_speed_cm_s = 0.0F;
                    output_speed_cm_s = 0.0F;
                    lost_updated_at = detected_at;
                }

                const ti_vision::TxMode tx_mode = web_server
                                                      ? web_server->tx_mode()
                                                      : ti_vision::TxMode::Tracking;
                const std::int8_t target_control = web_server
                                                       ? web_server->target_control()
                                                       : static_cast<std::int8_t>(options.target_control);
                bool transmitting = false;
                if (publisher) {
                    publisher->set_target_control(target_control);
                    if (tx_mode == ti_vision::TxMode::Manual) {
                        sent_position_cm = web_server->manual_cm();
                        sent_speed_cm_s = 0.0F;
                        publisher->publish(
                            sent_position_cm,
                            sent_speed_cm_s,
                            captured_ns);
                        transmitting = true;
                    } else if (tx_mode == ti_vision::TxMode::Tracking &&
                               (result.valid || last_direction != 0)) {
                        sent_position_cm = output_position_cm;
                        sent_speed_cm_s = output_speed_cm_s;
                        publisher->publish(
                            sent_position_cm,
                            sent_speed_cm_s,
                            captured_ns);
                        transmitting = true;
                    } else if (tx_mode == ti_vision::TxMode::Stopped &&
                               last_tx_mode != ti_vision::TxMode::Stopped) {
                        publisher->invalidate();
                    }
                }
                last_tx_mode = tx_mode;

                if (web_server) {
                    const double elapsed =
                        std::chrono::duration<double>(detected_at - started).count();
                    const std::uint64_t decoded = decoded_frames.load();
                    web_server->update(
                        local_y.data(),
                        local_y.size(),
                        detector.input_width(),
                        detector.input_height(),
                        result,
                        measured_position_cm,
                        raw_speed_cm_s,
                        output_speed_cm_s,
                        sent_position_cm,
                        sent_speed_cm_s,
                        transmitting,
                        elapsed > 0.0 ? decoded / elapsed : 0.0,
                        elapsed > 0.0 ? inferred_frames / elapsed : 0.0,
                        decoded,
                        inferred_frames,
                        decoded > inferred_frames ? decoded - inferred_frames : 0,
                        decoder_resets.load(),
                        publisher ? publisher->sent_frames() : 0,
                        publisher ? publisher->deadline_misses() : 0);
                }

                if (detected_at >= report_at) {
                    const double elapsed = std::chrono::duration<double>(detected_at - started).count();
                    std::cout << std::fixed << std::setprecision(2)
                              << "t=" << elapsed << "s decode=" << decoded_frames.load() / elapsed
                              << " infer=" << inferred_frames / elapsed
                              << " latest_drop="
                              << (decoded_frames.load() > inferred_frames
                                      ? decoded_frames.load() - inferred_frames
                                      : 0)
                              << " drained=" << kernel_drops.load()
                              << " decoder_resets=" << decoder_resets.load();
                    if (last_detection.valid) {
                        std::cout << std::setprecision(3)
                                  << " cx=" << last_detection.cx
                                  << " score=" << last_detection.score
                                  << " pos=" << output_position_cm << "cm"
                                  << " speed=" << output_speed_cm_s << "cm/s";
                    } else if (last_direction != 0) {
                        std::cout << std::setprecision(3)
                                  << " ball=lost direction="
                                  << (last_direction > 0 ? "left" : "right")
                                  << " pos=" << output_position_cm << "cm"
                                  << " speed=" << output_speed_cm_s << "cm/s";
                    } else {
                        std::cout << " ball=lost direction=unknown";
                    }
                    if (publisher) {
                        std::cout << " tx=" << publisher->sent_frames() / elapsed
                                  << "Hz miss=" << publisher->deadline_misses();
                    }
                    std::cout << '\n';
                    report_at = detected_at + std::chrono::seconds(1);
                }
            }
        } catch (...) {
            running = false;
            mailbox_ready.notify_all();
            if (capture_thread.joinable()) {
                capture_thread.join();
            }
            if (publisher) {
                publisher->stop();
            }
            if (web_server) {
                web_server->stop();
            }
            throw;
        }

        running = false;
        mailbox_ready.notify_all();
        if (capture_thread.joinable()) {
            capture_thread.join();
        }
        if (publisher) {
            publisher->stop();
        }
        if (web_server) {
            web_server->stop();
        }
        if (capture_error) {
            std::rethrow_exception(capture_error);
        }

        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::cout << std::fixed << std::setprecision(3)
                  << "summary elapsed=" << elapsed
                  << "s decoded=" << decoded_frames.load()
                  << " decode_fps=" << decoded_frames.load() / elapsed
                  << " inferred=" << inferred_frames
                  << " infer_fps=" << inferred_frames / elapsed
                  << " detected=" << detected_frames
                  << " drained=" << kernel_drops.load()
                  << " decoder_resets=" << decoder_resets.load();
        if (publisher) {
            std::cout << " tx=" << publisher->sent_frames()
                      << " tx_hz=" << publisher->sent_frames() / elapsed
                      << " deadline_miss=" << publisher->deadline_misses();
        }
        std::cout << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
