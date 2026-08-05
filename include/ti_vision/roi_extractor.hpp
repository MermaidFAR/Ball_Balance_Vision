#pragma once

#include <cstdint>
#include <vector>

#include "ti_vision/types.hpp"

namespace ti_vision {

void extract_nv12_roi(
    const DecodedFrameView& source,
    int crop_x,
    int crop_y,
    int crop_width,
    int crop_height,
    int output_width,
    int output_height,
    std::vector<std::uint8_t>& output_y,
    std::vector<std::uint8_t>& output_uv);

}
