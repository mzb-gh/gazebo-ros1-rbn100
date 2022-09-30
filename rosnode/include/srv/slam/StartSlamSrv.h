/*
 * StartSlamSrv.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include "../../msg/slam/StartInfo.h"
namespace slam_srvs {

struct StartSlamSrv
{
    StartSlamSrv(){}
    struct Request
    {
        slam_msgs::SlamStartInfo s_info;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace slam_srvs 