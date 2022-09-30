#pragma once

namespace nav_srvs
{
struct SelectModeNavSrv
{
    struct Request
    {
        // 1 tracking; 2 point_to_point
        int mode = 1;
    } request;

    struct Response
    {
        bool success;
    } response;
};
} // namespace nav_srvs
