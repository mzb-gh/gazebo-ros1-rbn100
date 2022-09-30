#pragma once

namespace topo_srvs
{
struct DeInitTopoSrv
{
    struct Request
    {
    } request;

    struct Response
    {
        std::uint8_t success; // 0代表成功, 其他代表失败
    } response;
};
} // namespace topo_srv
