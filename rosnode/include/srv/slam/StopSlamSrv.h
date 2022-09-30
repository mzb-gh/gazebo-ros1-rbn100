/*
 * StopSlamSrv.h
 *
 *  Created on: Aug 10, 2021
 *      Author: ubuntu
 */

#pragma once
#include "../../msg/slam/EndInfo.h"
namespace slam_srvs {

struct StopSlamSrv
{
    StopSlamSrv(){}
    struct Request
    {
        slam_msgs::SlamEndInfo s_end_info;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

}// namespace slam_srvs 