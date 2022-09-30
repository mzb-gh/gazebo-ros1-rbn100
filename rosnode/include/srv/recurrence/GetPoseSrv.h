/*
 * GetPoseSrv.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Liuqiaojun
 */

#pragma once

#include "../../msg/recurrence/Result.h"

namespace recurrence_srvs {

struct GetPoseSrv
{
    GetPoseSrv() {}

    struct Request
    {

    } request;

    struct Response
    {
        recurrence_msgs::Result s_pose;
        bool s_success;
    } response;

};

} //namespace recurrence_srvs