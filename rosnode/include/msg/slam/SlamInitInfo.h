#pragma once
#include <cstdint>
#include <string>
#include "../../msg/slam/ElevatorInfo.h"
namespace slam_msgs {

struct SlamInitInfo {
    std::uint64_t s_timestamp; 
    std::string s_param_path;
    std::string s_log_path;
};

}  // namespace slam_msgs