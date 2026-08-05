#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "ti_vision/bpu_detector.hpp"
#include "ti_vision/mjpeg_decoder.hpp"
#include "ti_vision/roi_extractor.hpp"

namespace {

std::vector<std::uint8_t> read_file(const char* path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error(std::string("cannot open ") + path);
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

}

int main(int argc, char** argv) {
    try {
        if (argc < 3) {
            std::cerr << "usage: " << argv[0] << " MODEL JPEG [JPEG...]\n";
            return 2;
        }

        ti_vision::BpuDetector detector(argv[1], 0.10F);
        ti_vision::MjpegDecoder decoder(1920, 1080);
        std::vector<std::uint8_t> roi_y;
        std::vector<std::uint8_t> roi_uv;
        float sum_x = 0.0F;
        int valid_count = 0;

        for (int index = 2; index < argc; ++index) {
            const auto jpeg = read_file(argv[index]);
            const auto decoded = decoder.decode(jpeg, static_cast<std::uint64_t>(index - 2));
            ti_vision::extract_nv12_roi(
                decoded, 0, 400, 1920, 288, 640, 96, roi_y, roi_uv);
            decoder.release_output();

            const ti_vision::DecodedFrameView roi{
                roi_y.data(), roi_uv.data(), 640, 96, 640};
            const auto detection = detector.infer(
                roi, 0, static_cast<std::uint64_t>(index - 2), 0);
            std::cout << argv[index] << " valid=" << detection.valid
                      << std::fixed << std::setprecision(3)
                      << " cx=" << detection.cx
                      << " cy=" << detection.cy
                      << " score=" << detection.score << '\n';
            if (detection.valid) {
                sum_x += detection.cx;
                ++valid_count;
            }
        }

        if (valid_count == 0) {
            throw std::runtime_error("no valid detections");
        }
        std::cout << std::fixed << std::setprecision(6)
                  << "mean_cx=" << sum_x / static_cast<float>(valid_count)
                  << " valid=" << valid_count << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
