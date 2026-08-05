#include "ti_vision/v4l2_camera.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <utility>

namespace ti_vision {
namespace {

void xioctl(int fd, unsigned long request, void* argument, const char* operation) {
    int result = 0;
    do {
        result = ioctl(fd, request, argument);
    } while (result < 0 && errno == EINTR);
    if (result < 0) {
        throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
    }
}

std::uint64_t monotonic_ns() {
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        throw std::runtime_error(std::string("clock_gettime: ") + std::strerror(errno));
    }
    return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
           static_cast<std::uint64_t>(value.tv_nsec);
}

}

V4L2Camera::V4L2Camera(std::string device, int width, int height, int fps, int buffer_count)
    : device_(std::move(device)), width_(width), height_(height) {
    if (width <= 0 || height <= 0 || fps <= 0 || buffer_count < 2) {
        throw std::invalid_argument("invalid camera configuration");
    }

    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        throw std::runtime_error("open " + device_ + ": " + std::strerror(errno));
    }

    try {
        v4l2_capability capability{};
        xioctl(fd_, VIDIOC_QUERYCAP, &capability, "VIDIOC_QUERYCAP");
        const std::uint32_t caps = capability.device_caps != 0 ? capability.device_caps : capability.capabilities;
        if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0 || (caps & V4L2_CAP_STREAMING) == 0) {
            throw std::runtime_error("camera does not support streaming capture");
        }

        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = static_cast<std::uint32_t>(width_);
        format.fmt.pix.height = static_cast<std::uint32_t>(height_);
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        format.fmt.pix.field = V4L2_FIELD_ANY;
        xioctl(fd_, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT");
        if (format.fmt.pix.width != static_cast<std::uint32_t>(width_) ||
            format.fmt.pix.height != static_cast<std::uint32_t>(height_) ||
            format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
            throw std::runtime_error("camera rejected 640x480 MJPEG mode");
        }

        v4l2_streamparm parameters{};
        parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parameters.parm.capture.timeperframe.numerator = 1;
        parameters.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(fps);
        xioctl(fd_, VIDIOC_S_PARM, &parameters, "VIDIOC_S_PARM");
        const auto& interval = parameters.parm.capture.timeperframe;
        if (interval.numerator == 0 || interval.denominator / interval.numerator < static_cast<std::uint32_t>(fps - 1)) {
            throw std::runtime_error("camera rejected requested frame rate");
        }

        v4l2_requestbuffers request{};
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        request.count = static_cast<std::uint32_t>(buffer_count);
        xioctl(fd_, VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS");
        if (request.count < 2) {
            throw std::runtime_error("camera allocated fewer than two buffers");
        }

        buffers_.resize(request.count);
        for (std::uint32_t index = 0; index < request.count; ++index) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            xioctl(fd_, VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF");
            void* address = mmap(nullptr, buffer.length, PROT_READ, MAP_SHARED, fd_, buffer.m.offset);
            if (address == MAP_FAILED) {
                throw std::runtime_error(std::string("mmap: ") + std::strerror(errno));
            }
            buffers_[index] = {address, buffer.length};
            xioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
        streaming_ = true;
    } catch (...) {
        close_camera();
        throw;
    }
}

V4L2Camera::~V4L2Camera() {
    close_camera();
}

void V4L2Camera::set_exposure_100us(int exposure_100us) {
    if (exposure_100us < 0) {
        return;
    }
    v4l2_control automatic{};
    automatic.id = V4L2_CID_EXPOSURE_AUTO;
    automatic.value = V4L2_EXPOSURE_MANUAL;
    xioctl(fd_, VIDIOC_S_CTRL, &automatic, "disable auto exposure");

    v4l2_control exposure{};
    exposure.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    exposure.value = exposure_100us;
    xioctl(fd_, VIDIOC_S_CTRL, &exposure, "set exposure");
}

CapturedFrame V4L2Camera::capture_latest(int timeout_ms) {
    pollfd descriptor{fd_, POLLIN, 0};
    int ready = 0;
    do {
        ready = poll(&descriptor, 1, timeout_ms);
    } while (ready < 0 && errno == EINTR);
    if (ready == 0) {
        throw std::runtime_error("camera timeout");
    }
    if (ready < 0) {
        throw std::runtime_error(std::string("poll: ") + std::strerror(errno));
    }

    CapturedFrame latest;
    std::uint32_t drained = 0;
    for (;;) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        int result = ioctl(fd_, VIDIOC_DQBUF, &buffer);
        if (result < 0) {
            if (errno == EAGAIN) {
                break;
            }
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::string("VIDIOC_DQBUF: ") + std::strerror(errno));
        }

        if (buffer.index >= buffers_.size() || buffer.bytesused == 0 ||
            buffer.bytesused > buffers_[buffer.index].length) {
            ioctl(fd_, VIDIOC_QBUF, &buffer);
            throw std::runtime_error("camera returned an invalid buffer");
        }

        const auto* begin = static_cast<const std::uint8_t*>(buffers_[buffer.index].address);
        latest.jpeg.assign(begin, begin + buffer.bytesused);
        latest.sequence = buffer.sequence;
        latest.captured_ns = monotonic_ns();
        ++drained;
        xioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
    }

    if (latest.jpeg.empty()) {
        throw std::runtime_error("camera became unreadable after poll");
    }
    latest.drained_frames = drained;
    return latest;
}

void V4L2Camera::close_camera() noexcept {
    if (streaming_ && fd_ >= 0) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }
    for (auto& buffer : buffers_) {
        if (buffer.address != nullptr && buffer.address != MAP_FAILED) {
            munmap(buffer.address, buffer.length);
            buffer.address = nullptr;
        }
    }
    buffers_.clear();
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

}
