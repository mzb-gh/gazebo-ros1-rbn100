#pragma once

#include <cstdint>

namespace nav_msgs {

struct SpeedControl
    {
        int Id;/*帧数*/
        double Speed_Linear;/*线速度*/
        double Speed_Angle;/*角速度*/
        std::uint64_t TimeStamp;
    };

}
