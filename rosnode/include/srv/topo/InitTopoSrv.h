#pragma once

#include <string>

namespace topo_srvs
{
struct InitTopoSrv
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
} // namespace topo_srv
