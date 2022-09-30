#pragma once

#include <string>

namespace nav_srvs
{
struct InitNavSrv
{
    struct Request
    {
        std::string path;
    } request;

    struct Response
    {
        bool success;
    } response;
};
} // namespace nav_srv
