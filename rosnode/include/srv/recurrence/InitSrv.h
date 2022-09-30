/*
 * InitSrv.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include "../../msg/recurrence/InitInfo.h"
namespace recurrence_srvs {

struct InitSrv
{
    InitSrv(){}
    struct Request
    {
        recurrence_msgs::InitInfo info;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace recurrence_srvs 