#pragma once

#include <cstdint>
#include <string>
// 纹理亮度控制
namespace sensor_srvs
{
  struct SysTimeSrv
  {
    SysTimeSrv() {}
    struct Request
    {
      uint64_t time;    //单位 us
    } request;

    struct Response
    {
      bool result; // true 成功， false 失败
      std::uint64_t timeStamp;
    } response;
  };
}