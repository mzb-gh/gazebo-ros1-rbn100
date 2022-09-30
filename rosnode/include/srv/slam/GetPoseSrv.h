/*
 * GetPoseSrv.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once

#include "../../msg/slam/SlamResult.h"

namespace slam_srvs {

struct GetPoseSrv
{
    GetPoseSrv() {}

    struct Request
    {
        std::uint64_t s_timestamp;

    } request;

    struct Response
    {
        slam_msgs::SlamResult s_pose;
        bool s_success;
    } response;

};

} //namespace slam_srvs