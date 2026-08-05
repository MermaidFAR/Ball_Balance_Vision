#include "ti_vision/recording_store.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ti_vision {
namespace {

constexpr std::size_t kMaximumQueuedFrames = 32;
constexpr std::uint64_t kMaximumRecordingMs = 5ULL * 60ULL * 1000ULL;

#pragma pack(push, 1)
struct FileHeader {
    char magic[8];
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t fps;
    std::uint32_t reserved;
    std::uint64_t started_unix_ms;
};

struct FrameHeader {
    std::uint64_t elapsed_ms;
    float measured_cm;
    float sent_cm;
    std::int8_t target_control;
    std::uint8_t detected;
    std::uint16_t reserved;
    float cx;
    float cy;
    float width;
    float height;
    float score;
};
#pragma pack(pop)

static_assert(sizeof(FileHeader) == 32, "unexpected recording file header size");
static_assert(sizeof(FrameHeader) == 40, "unexpected recording frame header size");

std::uint64_t steady_ms() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint64_t unix_ms() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string make_recording_id(std::uint64_t timestamp_ms) {
    const std::time_t seconds = static_cast<std::time_t>(timestamp_ms / 1000ULL);
    std::tm local{};
    if (const std::tm* converted = std::localtime(&seconds)) {
        local = *converted;
    }
    std::ostringstream id;
    id << std::put_time(&local, "%Y%m%d_%H%M%S")
       << '_' << std::setw(3) << std::setfill('0') << timestamp_ms % 1000ULL;
    return id.str();
}

FileHeader make_header(int width, int height, int fps, std::uint64_t started_unix_ms) {
    FileHeader header{};
    const char magic[8] = {'T', 'I', 'V', 'R', 'E', 'C', '1', '\0'};
    std::memcpy(header.magic, magic, sizeof(magic));
    header.width = static_cast<std::uint32_t>(width);
    header.height = static_cast<std::uint32_t>(height);
    header.fps = static_cast<std::uint32_t>(fps);
    header.started_unix_ms = started_unix_ms;
    return header;
}

bool header_valid(const FileHeader& header) {
    return std::memcmp(header.magic, "TIVREC1", 7) == 0 &&
           header.width > 0 && header.height > 0 && header.fps > 0;
}

FrameHeader make_frame_header(const RecordedFrame& frame) {
    FrameHeader header{};
    header.elapsed_ms = frame.elapsed_ms;
    header.measured_cm = frame.measured_cm;
    header.sent_cm = frame.sent_cm;
    header.target_control = frame.target_control;
    header.detected = frame.detection.valid ? 1U : 0U;
    header.cx = frame.detection.cx;
    header.cy = frame.detection.cy;
    header.width = frame.detection.width;
    header.height = frame.detection.height;
    header.score = frame.detection.score;
    return header;
}

}  // namespace

RecordingStore::RecordingStore(std::string directory, int width, int height, int fps)
    : directory_(std::move(directory)), width_(width), height_(height), fps_(fps) {
    if (directory_.empty() || width <= 0 || height <= 0 || fps <= 0 || fps > 30) {
        throw std::invalid_argument("invalid recording store configuration");
    }
}

RecordingStore::~RecordingStore() {
    shutdown();
}

void RecordingStore::start_worker() {
    if (worker_running_.exchange(true)) {
        return;
    }
    std::filesystem::create_directories(directory_);
    writer_thread_ = std::thread(&RecordingStore::writer_loop, this);
}

void RecordingStore::shutdown() {
    if (!worker_running_.exchange(false)) {
        return;
    }
    stop_recording();
    ready_.notify_all();
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
}

bool RecordingStore::start_recording(std::string& id, std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!worker_running_.load()) {
        error = "记录线程未运行";
        return false;
    }
    if (recording_.load()) {
        error = "当前已有测试正在记录";
        return false;
    }
    const std::uint64_t started_unix_ms = unix_ms();
    active_id_ = make_recording_id(started_unix_ms);
    recording_started_steady_ms_ = steady_ms();
    dropped_frames_ = 0;
    recording_ = true;
    QueueItem item;
    item.type = ItemType::Start;
    item.id = active_id_;
    item.started_unix_ms = started_unix_ms;
    queue_.push_back(std::move(item));
    id = active_id_;
    ready_.notify_one();
    return true;
}

void RecordingStore::stop_recording() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recording_.exchange(false)) {
        return;
    }
    QueueItem item;
    item.type = ItemType::Stop;
    item.id = active_id_;
    queue_.push_back(std::move(item));
    active_id_.clear();
    ready_.notify_one();
}

void RecordingStore::enqueue(
    const std::uint8_t* y,
    std::size_t y_size,
    float measured_cm,
    float sent_cm,
    std::int8_t target_control,
    const Detection& detection) {
    if (!recording_.load(std::memory_order_acquire) || y == nullptr ||
        y_size != static_cast<std::size_t>(width_ * height_)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recording_.load()) {
        return;
    }
    const std::uint64_t elapsed_ms = steady_ms() - recording_started_steady_ms_;
    if (elapsed_ms >= kMaximumRecordingMs) {
        recording_ = false;
        QueueItem stop;
        stop.type = ItemType::Stop;
        stop.id = active_id_;
        queue_.push_back(std::move(stop));
        active_id_.clear();
        ready_.notify_one();
        return;
    }
    const std::size_t queued_frames = static_cast<std::size_t>(std::count_if(
        queue_.begin(), queue_.end(),
        [](const QueueItem& item) { return item.type == ItemType::Frame; }));
    if (queued_frames >= kMaximumQueuedFrames) {
        ++dropped_frames_;
        return;
    }
    QueueItem item;
    item.type = ItemType::Frame;
    item.frame.y.assign(y, y + y_size);
    item.frame.elapsed_ms = elapsed_ms;
    item.frame.measured_cm = measured_cm;
    item.frame.sent_cm = sent_cm;
    item.frame.target_control = target_control;
    item.frame.detection = detection;
    queue_.push_back(std::move(item));
    ready_.notify_one();
}

bool RecordingStore::recording() const noexcept {
    return recording_.load(std::memory_order_acquire);
}

std::string RecordingStore::active_id() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_id_;
}

std::uint64_t RecordingStore::dropped_frames() const noexcept {
    return dropped_frames_.load();
}

std::vector<RecordingInfo> RecordingStore::list() const {
    std::vector<RecordingInfo> recordings;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator(directory_, error)) {
        if (error || !entry.is_regular_file() || entry.path().extension() != ".tivr") {
            continue;
        }
        std::ifstream stream(entry.path(), std::ios::binary);
        FileHeader header{};
        stream.read(reinterpret_cast<char*>(&header), sizeof(header));
        if (!stream || !header_valid(header)) {
            continue;
        }
        std::error_code size_error;
        const std::uint64_t bytes = entry.file_size(size_error);
        if (size_error || bytes < sizeof(FileHeader)) {
            continue;
        }
        const std::uint64_t frame_size = sizeof(FrameHeader) +
                                         static_cast<std::uint64_t>(header.width) * header.height;
        const std::uint64_t frame_count = (bytes - sizeof(FileHeader)) / frame_size;
        std::uint64_t duration_ms = 0;
        if (frame_count > 0) {
            stream.seekg(static_cast<std::streamoff>(
                sizeof(FileHeader) + (frame_count - 1) * frame_size));
            FrameHeader last{};
            stream.read(reinterpret_cast<char*>(&last), sizeof(last));
            if (stream) {
                duration_ms = last.elapsed_ms;
            }
        }
        RecordingInfo info;
        info.id = entry.path().stem().string();
        info.started_unix_ms = header.started_unix_ms;
        info.frame_count = frame_count;
        info.duration_ms = duration_ms;
        info.width = static_cast<int>(header.width);
        info.height = static_cast<int>(header.height);
        info.fps = static_cast<int>(header.fps);
        info.bytes = bytes;
        recordings.push_back(std::move(info));
    }
    std::sort(recordings.begin(), recordings.end(), [](const auto& left, const auto& right) {
        return left.started_unix_ms > right.started_unix_ms;
    });
    return recordings;
}

bool RecordingStore::read_frame(
    const std::string& id,
    std::uint64_t index,
    RecordedFrame& frame,
    std::string& error) const {
    if (!valid_id(id)) {
        error = "记录编号无效";
        return false;
    }
    const std::filesystem::path path = std::filesystem::path(directory_) / (id + ".tivr");
    std::ifstream stream(path, std::ios::binary);
    FileHeader header{};
    stream.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!stream || !header_valid(header)) {
        error = "记录不存在或格式损坏";
        return false;
    }
    const std::uint64_t image_size = static_cast<std::uint64_t>(header.width) * header.height;
    const std::uint64_t frame_size = sizeof(FrameHeader) + image_size;
    std::error_code size_error;
    const std::uint64_t bytes = std::filesystem::file_size(path, size_error);
    const std::uint64_t frame_count = size_error || bytes < sizeof(FileHeader)
                                          ? 0
                                          : (bytes - sizeof(FileHeader)) / frame_size;
    if (index >= frame_count) {
        error = "回放帧超出范围";
        return false;
    }
    stream.seekg(static_cast<std::streamoff>(sizeof(FileHeader) + index * frame_size));
    FrameHeader frame_header{};
    stream.read(reinterpret_cast<char*>(&frame_header), sizeof(frame_header));
    frame.y.resize(static_cast<std::size_t>(image_size));
    stream.read(reinterpret_cast<char*>(frame.y.data()),
                static_cast<std::streamsize>(frame.y.size()));
    if (!stream) {
        error = "回放帧读取失败";
        return false;
    }
    frame.elapsed_ms = frame_header.elapsed_ms;
    frame.measured_cm = frame_header.measured_cm;
    frame.sent_cm = frame_header.sent_cm;
    frame.target_control = frame_header.target_control;
    frame.detection.valid = frame_header.detected != 0;
    frame.detection.cx = frame_header.cx;
    frame.detection.cy = frame_header.cy;
    frame.detection.width = frame_header.width;
    frame.detection.height = frame_header.height;
    frame.detection.score = frame_header.score;
    return true;
}

void RecordingStore::writer_loop() {
    std::ofstream stream;
    std::uint64_t frames_since_flush = 0;
    while (true) {
        QueueItem item;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait_for(lock, std::chrono::milliseconds(200), [&] {
                return !queue_.empty() || !worker_running_.load();
            });
            if (queue_.empty()) {
                if (!worker_running_.load()) {
                    break;
                }
                continue;
            }
            item = std::move(queue_.front());
            queue_.pop_front();
        }
        if (item.type == ItemType::Start) {
            if (stream.is_open()) {
                stream.close();
            }
            const std::filesystem::path path =
                std::filesystem::path(directory_) / (item.id + ".tivr");
            stream.open(path, std::ios::binary | std::ios::trunc);
            const FileHeader header = make_header(width_, height_, fps_, item.started_unix_ms);
            stream.write(reinterpret_cast<const char*>(&header), sizeof(header));
            stream.flush();
            frames_since_flush = 0;
        } else if (item.type == ItemType::Frame && stream.is_open()) {
            const FrameHeader header = make_frame_header(item.frame);
            stream.write(reinterpret_cast<const char*>(&header), sizeof(header));
            stream.write(reinterpret_cast<const char*>(item.frame.y.data()),
                         static_cast<std::streamsize>(item.frame.y.size()));
            if (++frames_since_flush >= static_cast<std::uint64_t>(fps_)) {
                stream.flush();
                frames_since_flush = 0;
            }
        } else if (item.type == ItemType::Stop && stream.is_open()) {
            stream.flush();
            stream.close();
            frames_since_flush = 0;
        }
    }
    if (stream.is_open()) {
        stream.flush();
        stream.close();
    }
}

bool RecordingStore::valid_id(const std::string& id) const {
    return !id.empty() && id.size() <= 64 &&
           std::all_of(id.begin(), id.end(), [](unsigned char character) {
               return std::isalnum(character) || character == '_' || character == '-';
           });
}

}  // namespace ti_vision
