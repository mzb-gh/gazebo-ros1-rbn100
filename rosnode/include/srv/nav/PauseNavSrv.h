#pragma once

namespace nav_srvs
{
struct PauseNavSrv
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
