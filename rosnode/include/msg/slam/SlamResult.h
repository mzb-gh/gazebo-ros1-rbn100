#pragma once

#include <cstdint>
#include <vector>

namespace slam_msgs {

enum SlamRunState
{
    S_NORMAL = 0,
    RELOCAL_SUCCESS = 1,
    CLOSURE_SUCCESS = 2,
    IN_RAMP
};

enum ElevatorState
{
    E_NORMAL = 0,
    IN_ELEVATOR = 1,
    OUT_ELEVATOR
};

struct SlamResult {
    std::uint64_t s_time;
    float s_rotation[4];  // w, x, y, z
    float s_position[3];// x, y, z
    SlamRunState s_state;
    ElevatorState s_elevator; 
};

}  // namespace slam_msgs
