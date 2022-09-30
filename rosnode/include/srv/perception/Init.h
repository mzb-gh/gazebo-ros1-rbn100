//////////////////////////////////////////////////////////////////////
///  @file     init.h
///  @brief    perception init service
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
#include "../../msg/perception/SenmaticMap.h"

namespace perception_srvs 
{
struct Init
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
