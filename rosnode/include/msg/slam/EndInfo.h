#pragma once
#include <cstdint>
#include <string>
#include <cstring>

namespace slam_msgs {

struct PoseEnd
{
    float s_translation[3];
    float s_rotation[4];
    PoseEnd()
    {
        memset(s_translation,0,sizeof(s_translation));
        memset(s_rotation,0,sizeof(s_rotation));
    }
};
struct SlamEndInfo {
    std::uint64_t s_timestamp; 
    std::string s_map_path;
    PoseEnd s_end_pose;
};

}  // namespace slam_msgs