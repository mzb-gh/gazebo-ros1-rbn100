/*
 * GetHighReflectionPoint.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once

#include <vector>

namespace slam_srvs {
struct RampCoordinate
{
    int s_x;
    int s_y;
};
struct GetRampPointSrv
{
    GetRampPointSrv() {}

    struct Request
    {
    } request;

    struct Response
    {
        std::vector<RampCoordinate> s_ramp_points;
    } response;

};

} //namespace slam_srvs