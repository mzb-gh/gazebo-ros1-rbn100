#pragma once

#include <cstdint>
#include <vector>

namespace slam_msgs {

struct grid
{
    int s_x;
    int s_y;
    int8_t s_value;
};

enum MapState
{
    TOTAL = 0,
    PART = 1
};

struct OccupancyGrid
{
    std::uint64_t s_time;
    int s_width;
    int s_height;
    float s_resolution;
    MapState s_map_state;
    std::vector<grid> s_grid_data;
};

}  // namespace slam_msgs
