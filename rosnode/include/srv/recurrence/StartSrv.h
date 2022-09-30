/*
 * StartSrv.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Liuqiaojun
 */
#pragma once
namespace recurrence_srvs {

struct StartSrv
{
    StartSrv(){}
    struct Request
    {
    } request;

    struct Response
    {
        bool s_success;
    } response;


};

} //namespace recurrence_srvs 