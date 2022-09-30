/*
 * WakeupSlamSrv.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include <cstdint>

namespace slam_srvs {

struct WakeupSlamSrv
{
    WakeupSlamSrv(){}
    struct Request
    {
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace slam_srvs 