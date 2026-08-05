#include "ti_vision/bpu_detector.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <dnn/hb_sys.h>

namespace ti_vision {
namespace {

void require_dnn(int result, const char* operation) {
    if (result != 0) {
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(result));
    }
}

}

BpuDetector::BpuDetector(std::string model_path, float score_threshold)
    : model_path_(std::move(model_path)), score_threshold_(score_threshold) {
    if (score_threshold < 0.0F || score_threshold > 1.0F) {
        throw std::invalid_argument("invalid score threshold");
    }
    try {
        const char* path = model_path_.c_str();
        require_dnn(hbDNNInitializeFromFiles(&packed_handle_, &path, 1), "hbDNNInitializeFromFiles");

        const char** names = nullptr;
        int name_count = 0;
        require_dnn(hbDNNGetModelNameList(&names, &name_count, packed_handle_), "hbDNNGetModelNameList");
        if (name_count != 1) {
            throw std::runtime_error("expected exactly one model");
        }
        require_dnn(hbDNNGetModelHandle(&model_handle_, packed_handle_, names[0]), "hbDNNGetModelHandle");

        int input_count = 0;
        int output_count = 0;
        require_dnn(hbDNNGetInputCount(&input_count, model_handle_), "hbDNNGetInputCount");
        require_dnn(hbDNNGetOutputCount(&output_count, model_handle_), "hbDNNGetOutputCount");
        if (input_count != 1 || output_count != 1) {
            throw std::runtime_error("expected one model input and one output");
        }

        require_dnn(hbDNNGetInputTensorProperties(&input_.properties, model_handle_, 0),
                    "hbDNNGetInputTensorProperties");
        input_height_ = input_.properties.validShape.dimensionSize[2];
        input_width_ = input_.properties.validShape.dimensionSize[3];
        if (input_width_ <= 0 || input_height_ <= 0 || (input_width_ & 1) != 0 || (input_height_ & 1) != 0) {
            throw std::runtime_error("invalid model input shape");
        }

        require_dnn(hbDNNGetOutputTensorProperties(&output_.properties, model_handle_, 0),
                    "hbDNNGetOutputTensorProperties");
        if (output_.properties.tensorType != HB_DNN_TENSOR_TYPE_F32 ||
            output_.properties.validShape.numDimensions != 4 ||
            output_.properties.validShape.dimensionSize[1] != 5) {
            throw std::runtime_error("unexpected model output format");
        }
        prediction_count_ = output_.properties.validShape.dimensionSize[2];

        require_dnn(hbSysAllocCachedMem(&input_.sysMem[0], input_width_ * input_height_),
                    "allocate input Y");
        input_y_allocated_ = true;
        require_dnn(hbSysAllocCachedMem(&input_.sysMem[1], input_width_ * input_height_ / 2),
                    "allocate input UV");
        input_uv_allocated_ = true;
        require_dnn(hbSysAllocCachedMem(&output_.sysMem[0], output_.properties.alignedByteSize),
                    "allocate output");
        output_allocated_ = true;

        y_roi_.resize(static_cast<std::size_t>(input_width_ * input_height_));
        uv_roi_.resize(static_cast<std::size_t>(input_width_ * input_height_ / 2));
    } catch (...) {
        close_model();
        throw;
    }
}

BpuDetector::~BpuDetector() {
    close_model();
}

Detection BpuDetector::infer(
    const DecodedFrameView& frame,
    int crop_y,
    std::uint64_t sequence,
    std::uint64_t captured_ns) {
    if (frame.y == nullptr || frame.uv == nullptr || frame.width < input_width_ || frame.stride < input_width_ ||
        crop_y < 0 || (crop_y & 1) != 0 || crop_y + input_height_ > frame.height) {
        throw std::invalid_argument("decoded frame cannot satisfy model ROI");
    }

    for (int row = 0; row < input_height_; ++row) {
        std::memcpy(y_roi_.data() + static_cast<std::size_t>(row * input_width_),
                    frame.y + static_cast<std::size_t>((crop_y + row) * frame.stride),
                    static_cast<std::size_t>(input_width_));
    }
    const int uv_crop_y = crop_y / 2;
    for (int row = 0; row < input_height_ / 2; ++row) {
        std::memcpy(uv_roi_.data() + static_cast<std::size_t>(row * input_width_),
                    frame.uv + static_cast<std::size_t>((uv_crop_y + row) * frame.stride),
                    static_cast<std::size_t>(input_width_));
    }

    require_dnn(hbSysWriteMem(&input_.sysMem[0], reinterpret_cast<char*>(y_roi_.data()), y_roi_.size()),
                "write input Y");
    require_dnn(hbSysWriteMem(&input_.sysMem[1], reinterpret_cast<char*>(uv_roi_.data()), uv_roi_.size()),
                "write input UV");
    input_.properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
    input_.properties.tensorType = HB_DNN_IMG_TYPE_NV12_SEPARATE;
    input_.properties.validShape.numDimensions = 4;
    input_.properties.validShape.dimensionSize[0] = 1;
    input_.properties.validShape.dimensionSize[1] = 3;
    input_.properties.validShape.dimensionSize[2] = input_height_;
    input_.properties.validShape.dimensionSize[3] = input_width_;
    input_.properties.alignedShape = input_.properties.validShape;

    hbDNNTaskHandle_t task = nullptr;
    hbDNNTensor* output_pointer = &output_;
    hbDNNInferCtrlParam control;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
    require_dnn(hbDNNInfer(&task, &output_pointer, &input_, model_handle_, &control), "hbDNNInfer");
    try {
        require_dnn(hbDNNWaitTaskDone(task, 0), "hbDNNWaitTaskDone");
    } catch (...) {
        hbDNNReleaseTask(task);
        throw;
    }
    require_dnn(hbDNNReleaseTask(task), "hbDNNReleaseTask");
    require_dnn(hbSysFlushMem(&output_.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE), "invalidate output");

    const auto* values = static_cast<const float*>(output_.sysMem[0].virAddr);
    const float* scores = values + 4 * prediction_count_;
    const auto best = std::max_element(scores, scores + prediction_count_);
    const int index = static_cast<int>(best - scores);

    Detection detection;
    detection.score = *best;
    detection.sequence = sequence;
    detection.captured_ns = captured_ns;
    if (detection.score >= score_threshold_) {
        detection.valid = true;
        detection.cx = values[index];
        detection.cy = values[prediction_count_ + index];
        detection.width = values[2 * prediction_count_ + index];
        detection.height = values[3 * prediction_count_ + index];
    }
    return detection;
}

int BpuDetector::input_width() const noexcept {
    return input_width_;
}

int BpuDetector::input_height() const noexcept {
    return input_height_;
}

void BpuDetector::close_model() noexcept {
    if (output_allocated_) {
        hbSysFreeMem(&output_.sysMem[0]);
        output_allocated_ = false;
    }
    if (input_uv_allocated_) {
        hbSysFreeMem(&input_.sysMem[1]);
        input_uv_allocated_ = false;
    }
    if (input_y_allocated_) {
        hbSysFreeMem(&input_.sysMem[0]);
        input_y_allocated_ = false;
    }
    if (packed_handle_ != nullptr) {
        hbDNNRelease(packed_handle_);
        packed_handle_ = nullptr;
        model_handle_ = nullptr;
    }
}

}
