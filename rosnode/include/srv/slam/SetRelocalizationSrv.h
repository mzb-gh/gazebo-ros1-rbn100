/*
 * SetRelocalizationSrv.h
 *
 *  Created on: Jul 11, 2022
 *      Author: Liuqiaojun
 */
#pragma once
#include <cstdint>

namespace slam_srvs {
    
struct SetRelocalizationSrv
{
     SetRelocalizationSrv() {}
    struct Request
    {
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} // namespace slam_srvs