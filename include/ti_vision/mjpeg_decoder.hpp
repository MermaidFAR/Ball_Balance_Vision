#pragma once

#include <cstdint>
#include <vector>

#include <hb_media_codec.h>

#include "ti_vision/types.hpp"

namespace ti_vision {

class MjpegDecoder {
public:
    MjpegDecoder(int width, int height);
    ~MjpegDecoder();

    MjpegDecoder(const MjpegDecoder&) = delete;
    MjpegDecoder& operator=(const MjpegDecoder&) = delete;

    DecodedFrameView decode(const std::vector<std::uint8_t>& jpeg, std::uint64_t sequence);
    void release_output();

private:
    void close_decoder() noexcept;

    media_codec_context_t context_{};
    media_codec_buffer_t output_buffer_{};
    bool memory_open_ = false;
    bool initialized_ = false;
    bool started_ = false;
    bool output_held_ = false;
};

}
