//////////////////////////////////////////////////////////////////////
///  @file     stop.h
///  @brief    perception stop service
///  Details.
///
///  @author   sunhao
///  @version  1.0.0
///  @date     2022.03.31
///
///  revision statement:
//////////////////////////////////////////////////////////////////////

#pragma once
#include <cstdint>

namespace perception_srvs 
{
struct Stop
{
    struct Request
    {
        std::uint64_t time;
    } request;

    struct Response
    {
        bool status;
    } response;
};

}// namespace perception 
