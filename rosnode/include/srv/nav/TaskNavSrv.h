#pragma once
#include <vector>

namespace nav_srvs
{
struct TaskNavSrv
{
    struct POINT
    {
        float x;
        float y;
        float theta;
    };

    struct Request
    {
        std::vector<POINT> pt;
    } request;

    struct Response
    {
        bool success;
    } response;
};
} // namespace nav_srvs
