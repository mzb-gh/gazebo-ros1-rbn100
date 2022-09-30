#pragma once

#include <srv/topo/SetTopoMapSrv.h>
#include <string>
#include <vector>

namespace topo_srvs
{
struct GetPathViaUUIDSrv
{
    struct Request
    {
        std::vector<std::string> uuidList;
    } request;

    struct Response
    {
        // path;
        std::vector<IPoint> path;
        bool success;
    } response;
};
} // namespace topo_srvs
