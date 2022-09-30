#pragma once

#include <cstdint>

namespace nav_msgs
{
struct GetWorkLengthNTime
{
    bool isFirst;         // true: it's the first time, otherwise false
    std::uint32_t length; // m: The estimate of working path length
    std::uint64_t time;   // s: The estimate of working time
};

} // namespace nav_msgs
