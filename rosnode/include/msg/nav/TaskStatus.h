#pragma once

#include <cstdint>

namespace nav_msgs
{
enum Status
{
    TASK_FAILED,
    TASK_SUCCESSED,
};
struct TaskStatus
{
    Status taskStatus;
    std::uint64_t time; // us
};

} // namespace nav_msgs
