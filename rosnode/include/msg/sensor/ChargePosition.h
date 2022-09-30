#pragma once

#include <cstdint>

namespace sensors_msgs {

struct ChargePosition
{
    uint32_t frameId; // 帧号
    std::uint64_t timeStamp;
};

}