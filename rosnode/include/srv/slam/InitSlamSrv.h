/*
 * InitSlamSrv.h
 *
 *  Created on: Jun 20, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include "../../msg/slam/SlamInitInfo.h"
namespace slam_srvs {

struct InitSlamSrv
{
    InitSlamSrv(){}
    struct Request
    {
        slam_msgs::SlamInitInfo s_info;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace slam_srvs 