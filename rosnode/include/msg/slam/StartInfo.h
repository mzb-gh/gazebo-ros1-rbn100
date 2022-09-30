#pragma once
#include <cstdint>
#include <string>
#include "../../msg/slam/ElevatorInfo.h"
namespace slam_msgs {

enum RunMode {
        MANNUAL,
        AUTO,
        MANNUAL_ELEVATOR,
        AUTO_ELEVATOR
    };
struct SlamStartInfo {
    std::uint64_t s_timestamp; 
    RunMode s_run_mode;
    std::string s_param_path;
    std::string s_log_path;
    std::string s_map_path;
    PoseInit s_init_pose;
    ElevatorInfo s_elevator_info;
};

}  // namespace slam_msgs