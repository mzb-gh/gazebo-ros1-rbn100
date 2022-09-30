#pragma once

#include <string>

namespace topo_srvs
{
struct CreateTopoSrv
{
    struct Request
    {
        struct StaticMap
        {
        } staticMapItem;
        struct SitesList
        {
        } siteListItem;
        struct VirtualWall
        {
        } virtualWallItem;
    } request;

    struct Response
    {
        struct TopoMap
        {
        } topoMapItem;
        bool success;
    } response;
};
} // namespace topo_srvs
