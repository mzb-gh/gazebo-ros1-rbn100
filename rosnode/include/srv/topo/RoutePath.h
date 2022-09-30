#pragma once

#include <string>
#include <vector>

namespace topo_srvs
{
struct RoutePathSrv
{
    struct Request
    {
        std::string startSite;
        std::string endSite;
    } request;

    struct Response
    {
        std::vector<std::string> pathListWithUUID;
        bool success;
    } response;
};
} // namespace topo_srvs
