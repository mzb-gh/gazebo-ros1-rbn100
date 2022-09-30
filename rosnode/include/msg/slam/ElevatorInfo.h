#pragma once
#include <cstdint>
#include <string>
#include <cstring>
namespace slam_msgs {

enum ElevatorRunMode {
        FIRST = 0,
        SECOND
    };
struct PoseInit
{
    float s_translation[3];
    float s_rotation[4];
    PoseInit()
    {
        memset(s_translation,0,sizeof(s_translation));
        memset(s_rotation,0,sizeof(s_rotation));
    }
    };
struct ElevatorInfo
{
    ElevatorRunMode s_elevator_mode;
    PoseInit s_pose;
    std::string s_map_path;
    int s_elevator_num;//当前层数
    int s_elevator_serial_num;//接下来的层数
};


}  // namespace slam_msgs