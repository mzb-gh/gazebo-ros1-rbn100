/*
 * StopSrv.h
 *
 *  Created on: Jul 21, 2021
 *      Author: ubuntu
 */

#pragma once
#include <cstdint>
namespace recurrence_srvs {

struct StopSrv
{
    StopSrv(){}
    struct Request
    {
        uint64_t s_time;
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

}// namespace recurrence_srvs 