#pragma once

#include <opencv2/core.hpp>

namespace depth_msgs {
    struct DepthMatrix {
        std::uint64_t frame_id;
        std::uint64_t timestamp;

        enum {
            NONE,} flag;

        cv::Mat data;
        cv::Mat gray;
        float rotation[4];  // (w, x, y, z)
        float position[3];  // (x, y, z)
        cv::Mat trans;
    };
}


