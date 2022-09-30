#pragma once

#include <string>

namespace nav_srvs
{
struct DeInitNavSrv
{
    struct Request
    {
    } request;

    struct Response
    {
        std::uint8_t success; // 0代表成功, 其他代表失败
    } response;
};
} // namespace nav_srv
