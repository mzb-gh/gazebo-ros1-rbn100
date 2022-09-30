/*
 * PauseSlamSrv.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include <cstdint>

namespace slam_srvs {
    
struct PauseSlamSrv
{
     PauseSlamSrv() {}
    struct Request
    {
    } request;

    struct Response
    {
        std::uint64_t s_timestamp;
        bool s_success;
    } response;


};

} // namespace slam_srvs