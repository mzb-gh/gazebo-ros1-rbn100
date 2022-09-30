#pragma once

namespace nav_srvs
{
struct CancelNavSrv
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
