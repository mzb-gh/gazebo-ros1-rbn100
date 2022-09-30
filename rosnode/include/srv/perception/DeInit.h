//////////////////////////////////////////////////////////////////////
///  @file     init.h
///  @brief    perception init service
///  Details.
///
///  @author   donglijian
///  @version  1.0.0
///  @date     2022.06.16
///
///  revision statement:
//////////////////////////////////////////////////////////////////////

#pragma once
#include <cstdint>
#include "../../msg/perception/SenmaticMap.h"

namespace perception_srvs 
{
struct DeInit
{
    struct Request
    {
        std::string configPath;
    } request;

    struct Response
    {
        bool status;
    } response;
};

}// namespace perception 
