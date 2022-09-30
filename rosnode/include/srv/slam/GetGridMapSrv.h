/*
 * GetGridMap.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once

#include "../../msg/slam/OccupancyGrid.h"

namespace slam_srvs {

struct GetGridMapSrv
{
    GetGridMapSrv() {}

    struct Request
    {
    } request;

    struct Response
    {
        slam_msgs::OccupancyGrid s_grid_map;
    } response;

};

} //namespace slam_srvs