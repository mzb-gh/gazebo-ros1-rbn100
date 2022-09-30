#pragma once

#include <string>

namespace topo_srvs
{
struct SetGridMapSrv
{
    struct Request
    {
        std::string mapConfigPath;
        std::string gridMapPath;
    } request;

    struct Response
    {
        bool success;
    } response;
};
} // namespace topo_srvs
