//////////////////////////////////////////////////////////////////////
///  @file     load_map.h
///  @brief    perception LoadMap service
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
#include <string>

namespace perception_srvs 
{
struct LoadMap
{
    struct Request
    {
        std::string file;
    } request;

    struct Response
    {
        bool status;
    } response;
};

}// namespace perception 
