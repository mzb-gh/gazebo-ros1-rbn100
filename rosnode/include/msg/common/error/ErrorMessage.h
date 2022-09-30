/*
 * EulerAngle.h
 *
 *  Created on: Feb 23, 2021
 *      Author: ubuntu
 */

#include "../base/Time.h"
#include "ErrorCode.h"

#pragma once

namespace error_msgs {

enum ErrorState
{
    ON,
    OFF
};

struct ErrorMessage
{
    ErrorMessage() = default;
    ErrorMessage(base_msgs::Time stamp, ErrorCode code, ErrorState state)
        : stamp(stamp), code(code), state(state) {}
    base_msgs::Time stamp;
    ErrorCode code;
    ErrorState state;
};
}

