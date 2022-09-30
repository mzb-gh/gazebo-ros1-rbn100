#pragma once

namespace nav_srvs
{
struct ResumeNavSrv
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
