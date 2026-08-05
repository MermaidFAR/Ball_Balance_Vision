#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ti_vision/types.hpp"

namespace ti_vision {

class V4L2Camera {
public:
    V4L2Camera(std::string device, int width, int height, int fps, int buffer_count);
    ~V4L2Camera();

    V4L2Camera(const V4L2Camera&) = delete;
    V4L2Camera& operator=(const V4L2Camera&) = delete;

    void set_exposure_100us(int exposure_100us);
    CapturedFrame capture_latest(int timeout_ms);

private:
    struct MappedBuffer {
        void* address = nullptr;
        std::size_t length = 0;
    };

    void close_camera() noexcept;

    std::string device_;
    int fd_ = -1;
    int width_ = 0;
    int height_ = 0;
    bool streaming_ = false;
    std::vector<MappedBuffer> buffers_;
};

}
