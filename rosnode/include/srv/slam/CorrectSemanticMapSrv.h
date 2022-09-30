/*
 * CorrectSemanticMap.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once
#include "../../msg/perception/SenmaticMap.h"
namespace slam_srvs {

struct CorrectSemanticMapSrv
{
    CorrectSemanticMapSrv() {}

    struct Request
    {

    } request;

    struct Response
    {
        perception_msgs::SenmaticMap s_sen_map;
        bool s_success;
    } response;

};

} //namespace slam_srvs