/*
 * InitSrv.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include <cstdint>
namespace recurrence_srvs {

struct DeInitSrv
{
    DeInitSrv(){}
    struct Request
    {
        uint64_t s_time;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace recurrence_srvs 