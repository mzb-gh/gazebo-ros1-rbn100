#pragma once

#include <vector>
#include <cstdint>

namespace depth_msgs {

struct Point32 {
    float x;
    float y;
    float z;
};


struct DepthPointCloud {
    using Point = Point32;

    std::uint64_t frame_id;
    std::uint64_t timestamp;

    enum {
        NONE,
    } flag;

    float rotation[4];  // (w, x, y, z)
    float position[3];  // (x, y, z)

    struct label_point{
    std::vector<Point> points;
    std::vector<char> label;
    };

    Point center;
    std::vector<label_point> data;
};
}



