#pragma once
#include <cstdint>
#include "../../msg/common/error/ErrorMessage.h"
#include "../../msg/common/error/ErrorCode.h"
namespace slam_msgs {

struct SlamErrorInfo {
    error_msgs::ErrorMessage s_error_msg;
};

}  // namespace slam_msgs

