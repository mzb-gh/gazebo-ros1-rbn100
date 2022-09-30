#pragma once

#include <opencv2/core.hpp>

namespace depth_msgs {

struct CalibrationImage {
    std::uint64_t timestamp;
    cv::Mat left;
    cv::Mat right;
    float fillRate;
};
}



