#pragma once
#include <cstdint>
#include <string>
namespace recurrence_msgs {

struct InitInfo {
    std::uint64_t s_timestamp; 
    std::string s_param_path;
    std::string s_log_path;
};

}  // namespace recurrence_msgs