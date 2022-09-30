/*
 * GetHighReflectionPoint.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once

#include <vector>

namespace slam_srvs {
struct RefCoordinate
{
    int s_x;
    int s_y;
};
struct GetHighReflectionPointSrv
{
    GetHighReflectionPointSrv() {}

    struct Request
    {
    } request;

    struct Response
    {
        std::vector<RefCoordinate> s_ref_points;
    } response;

};

} //namespace slam_srvs