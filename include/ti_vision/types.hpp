#pragma once

#include <cstdint>
#include <vector>

namespace ti_vision {

struct CapturedFrame {
    std::vector<std::uint8_t> jpeg;
    std::uint64_t sequence = 0;
    std::uint64_t captured_ns = 0;
    std::uint32_t drained_frames = 0;
};

struct DecodedFrameView {
    const std::uint8_t* y = nullptr;
    const std::uint8_t* uv = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;
};

struct Detection {
    bool valid = false;
    float cx = 0.0F;
    float cy = 0.0F;
    float width = 0.0F;
    float height = 0.0F;
    float score = 0.0F;
    std::uint64_t sequence = 0;
    std::uint64_t captured_ns = 0;
};

}
