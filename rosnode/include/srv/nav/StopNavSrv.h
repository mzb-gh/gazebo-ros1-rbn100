#pragma once

namespace nav_srvs
{
struct StopNavSrv
{
    struct Request
    {
    } request;

    struct Response
    {
        bool success;
    } response;
};
} // namespace nav_srv
