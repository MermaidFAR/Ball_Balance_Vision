#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <dnn/hb_dnn.h>

#include "ti_vision/types.hpp"

namespace ti_vision {

class BpuDetector {
public:
    BpuDetector(std::string model_path, float score_threshold);
    ~BpuDetector();

    BpuDetector(const BpuDetector&) = delete;
    BpuDetector& operator=(const BpuDetector&) = delete;

    Detection infer(
        const DecodedFrameView& frame,
        int crop_y,
        std::uint64_t sequence,
        std::uint64_t captured_ns);

    int input_width() const noexcept;
    int input_height() const noexcept;

private:
    void close_model() noexcept;

    std::string model_path_;
    float score_threshold_ = 0.0F;
    hbPackedDNNHandle_t packed_handle_ = nullptr;
    hbDNNHandle_t model_handle_ = nullptr;
    hbDNNTensor input_{};
    hbDNNTensor output_{};
    bool input_y_allocated_ = false;
    bool input_uv_allocated_ = false;
    bool output_allocated_ = false;
    int input_width_ = 0;
    int input_height_ = 0;
    int prediction_count_ = 0;
    std::vector<std::uint8_t> y_roi_;
    std::vector<std::uint8_t> uv_roi_;
};

}
