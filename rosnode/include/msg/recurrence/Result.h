#pragma once

#include <cstdint>

namespace recurrence_msgs {

struct Result {
    std::uint64_t s_time;
    float s_rotation[4];  // w, x, y, z
    float s_position[3];// x, y, z
};

}  // namespace recurrence_msgs
