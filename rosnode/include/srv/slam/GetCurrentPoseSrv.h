/*
 * GetCurrentPoseSrv.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once

#include "../../msg/slam/SlamResult.h"

namespace slam_srvs {

struct GetCurrentPoseSrv
{
    GetCurrentPoseSrv() {}

    struct Request
    {

    } request;

    struct Response
    {
        slam_msgs::SlamResult s_pose;
    } response;

};

} //namespace slam_srvs