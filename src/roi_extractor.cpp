#include "ti_vision/roi_extractor.hpp"

#include <cstring>
#include <stdexcept>

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
    std::vector<std::uint8_t>& output_uv) {
    if (source.y == nullptr || source.uv == nullptr || source.stride < source.width ||
        crop_x < 0 || crop_y < 0 || crop_width <= 0 || crop_height <= 0 ||
        output_width <= 0 || output_height <= 0 ||
        crop_x + crop_width > source.width || crop_y + crop_height > source.height ||
        (crop_x & 1) != 0 || (crop_y & 1) != 0 ||
        (crop_width & 1) != 0 || (crop_height & 1) != 0 ||
        (output_width & 1) != 0 || (output_height & 1) != 0 ||
        crop_width % output_width != 0 || crop_height % output_height != 0) {
        throw std::invalid_argument("invalid NV12 crop/resize geometry");
    }

    const int scale_x = crop_width / output_width;
    const int scale_y = crop_height / output_height;
    if (scale_x != scale_y || scale_x <= 0) {
        throw std::invalid_argument("NV12 extractor requires equal integer scaling");
    }
    const int scale = scale_x;

    output_y.resize(static_cast<std::size_t>(output_width * output_height));
    output_uv.resize(output_y.size() / 2U);

    if (scale == 1) {
        for (int row = 0; row < output_height; ++row) {
            std::memcpy(
                output_y.data() + static_cast<std::size_t>(row * output_width),
                source.y + static_cast<std::size_t>((crop_y + row) * source.stride + crop_x),
                static_cast<std::size_t>(output_width));
        }
        for (int row = 0; row < output_height / 2; ++row) {
            std::memcpy(
                output_uv.data() + static_cast<std::size_t>(row * output_width),
                source.uv + static_cast<std::size_t>((crop_y / 2 + row) * source.stride + crop_x),
                static_cast<std::size_t>(output_width));
        }
        return;
    }

    const int area = scale * scale;
    for (int dst_y = 0; dst_y < output_height; ++dst_y) {
        for (int dst_x = 0; dst_x < output_width; ++dst_x) {
            unsigned int sum = 0;
            const int src_y = crop_y + dst_y * scale;
            const int src_x = crop_x + dst_x * scale;
            for (int y = 0; y < scale; ++y) {
                const auto* row = source.y + static_cast<std::size_t>((src_y + y) * source.stride + src_x);
                for (int x = 0; x < scale; ++x) {
                    sum += row[x];
                }
            }
            output_y[static_cast<std::size_t>(dst_y * output_width + dst_x)] =
                static_cast<std::uint8_t>((sum + area / 2) / area);
        }
    }

    const int uv_crop_y = crop_y / 2;
    for (int dst_y = 0; dst_y < output_height / 2; ++dst_y) {
        for (int dst_pair = 0; dst_pair < output_width / 2; ++dst_pair) {
            unsigned int sum_u = 0;
            unsigned int sum_v = 0;
            const int src_y = uv_crop_y + dst_y * scale;
            const int src_x = crop_x + dst_pair * 2 * scale;
            for (int y = 0; y < scale; ++y) {
                const auto* row = source.uv + static_cast<std::size_t>((src_y + y) * source.stride + src_x);
                for (int x = 0; x < scale; ++x) {
                    sum_u += row[2 * x];
                    sum_v += row[2 * x + 1];
                }
            }
            const std::size_t output_index =
                static_cast<std::size_t>(dst_y * output_width + dst_pair * 2);
            output_uv[output_index] = static_cast<std::uint8_t>((sum_u + area / 2) / area);
            output_uv[output_index + 1U] = static_cast<std::uint8_t>((sum_v + area / 2) / area);
        }
    }
}

}
