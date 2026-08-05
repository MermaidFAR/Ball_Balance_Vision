#include "ti_vision/mjpeg_decoder.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <hb_media_error.h>
#include <hb_mem_mgr.h>
#include <hbmem.h>

namespace ti_vision {
namespace {

void require_codec(int result, const char* operation) {
    if (result != 0) {
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(result));
    }
}

}

MjpegDecoder::MjpegDecoder(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("invalid decoder size");
    }
    try {
        require_codec(hb_mem_module_open(), "hb_mem_module_open");
        memory_open_ = true;

        context_.encoder = false;
        context_.codec_id = MEDIA_CODEC_ID_MJPEG;
        auto& params = context_.video_dec_params;
        params.feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
        params.pix_fmt = MC_PIXEL_FORMAT_NV12;
        params.bitstream_buf_size = static_cast<std::uint32_t>((width * height * 3 / 2 + 0x3FF) & ~0x3FF);
        params.bitstream_buf_count = 3;
        params.frame_buf_count = 3;
        params.mjpeg_dec_config.rot_degree = MC_CCW_0;
        params.mjpeg_dec_config.mir_direction = MC_DIRECTION_NONE;
        params.mjpeg_dec_config.frame_crop_enable = false;

        require_codec(hb_mm_mc_initialize(&context_), "hb_mm_mc_initialize");
        initialized_ = true;
        require_codec(hb_mm_mc_configure(&context_), "hb_mm_mc_configure");

        mc_av_codec_startup_params_t startup{};
        require_codec(hb_mm_mc_start(&context_, &startup), "hb_mm_mc_start");
        started_ = true;
    } catch (...) {
        close_decoder();
        throw;
    }
}

MjpegDecoder::~MjpegDecoder() {
    close_decoder();
}

DecodedFrameView MjpegDecoder::decode(const std::vector<std::uint8_t>& jpeg, std::uint64_t sequence) {
    if (jpeg.empty()) {
        throw std::invalid_argument("empty JPEG frame");
    }
    release_output();

    media_codec_buffer_t input{};
    input.type = MC_VIDEO_STREAM_BUFFER;
    require_codec(hb_mm_mc_dequeue_input_buffer(&context_, &input, 100), "dequeue decoder input");
    if (input.vstream_buf.size < jpeg.size()) {
        hb_mm_mc_queue_input_buffer(&context_, &input, 0);
        throw std::runtime_error("JPEG frame exceeds decoder bitstream buffer");
    }
    std::memcpy(input.vstream_buf.vir_ptr, jpeg.data(), jpeg.size());
    input.vstream_buf.size = static_cast<std::uint32_t>(jpeg.size());
    input.vstream_buf.stream_end = 0;
    input.vstream_buf.pts = static_cast<std::int64_t>(sequence);
    require_codec(hb_mm_mc_queue_input_buffer(&context_, &input, 100), "queue decoder input");

    media_codec_output_buffer_info_t info{};
    output_buffer_ = {};
    const int result = hb_mm_mc_dequeue_output_buffer(&context_, &output_buffer_, &info, 100);
    require_codec(result, "dequeue decoder output");
    output_held_ = true;
    if (output_buffer_.type != MC_VIDEO_FRAME_BUFFER || output_buffer_.vframe_buf.size == 0 ||
        output_buffer_.vframe_buf.vir_ptr[0] == nullptr || output_buffer_.vframe_buf.vir_ptr[1] == nullptr) {
        release_output();
        throw std::runtime_error("decoder returned an invalid NV12 frame");
    }

    const auto& frame = output_buffer_.vframe_buf;
    return {frame.vir_ptr[0], frame.vir_ptr[1], frame.width, frame.height, frame.stride};
}

void MjpegDecoder::release_output() {
    if (!output_held_) {
        return;
    }
    const int result = hb_mm_mc_queue_output_buffer(&context_, &output_buffer_, 0);
    output_held_ = false;
    output_buffer_ = {};
    require_codec(result, "release decoder output");
}

void MjpegDecoder::close_decoder() noexcept {
    if (output_held_) {
        hb_mm_mc_queue_output_buffer(&context_, &output_buffer_, 0);
        output_held_ = false;
    }
    if (started_) {
        hb_mm_mc_stop(&context_);
        started_ = false;
    }
    if (initialized_) {
        hb_mm_mc_release(&context_);
        initialized_ = false;
    }
    if (memory_open_) {
        hb_mem_module_close();
        memory_open_ = false;
    }
}

}
