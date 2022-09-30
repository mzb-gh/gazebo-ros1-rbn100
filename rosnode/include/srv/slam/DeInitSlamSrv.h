/*
 * InitSlamSrv.h
 *
 *  Created on: Jun 20, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include "../../msg/slam/ElevatorInfo.h"
namespace slam_srvs {

struct DeInitSlamSrv
{
    DeInitSlamSrv(){}
    struct Request
    {
        uint64_t s_time;
        slam_msgs::PoseInit s_pose;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace slam_srvs 