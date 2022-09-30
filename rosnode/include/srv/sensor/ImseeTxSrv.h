#pragma once

#include <cstdint>
#include <string>
// 纹理亮度控制
namespace sensor_srvs
{
  struct ImseeTxSrv
  {
    ImseeTxSrv() {}
    struct Request
    {
      int value;    //纹理亮度值
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      bool result; // true 连接成功， false 连接失败
      std::uint64_t timeStamp;
    } response;
  };
}