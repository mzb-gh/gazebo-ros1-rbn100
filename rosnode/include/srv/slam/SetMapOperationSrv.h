/*
 * SetMapOperation.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Liuqiaojun
 */

#pragma once

namespace slam_srvs {

enum MapOpType
{
    LOAD = 0,
    SAVE = 1,
    UPDATE = 2
};

struct SetMapOperationSrv
{
    SetMapOperationSrv() {}

    struct Request
    {
        MapOpType s_op_map;
    } request;

    struct Response
    {
        bool s_success;
    } response;

};

} //namespace slam_srvs