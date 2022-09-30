/*
 * StartDepthSrv.h
 *
 *  Created on: Aug 10, 2021
 *      Author: ubuntu
 */

#pragma once
#include <cstdint>

namespace depth_srvs {

struct PauseDepthSrv
{

    struct Request
    {
        std::uint64_t timestamp;
    } request;

    struct Response
    {
        bool success;
    } response;

};

} //namespace depth_srvs 
