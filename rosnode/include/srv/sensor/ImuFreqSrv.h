#pragma once

#include <cstdint>
#include <string>
// 纹理亮度控制
namespace sensor_srvs
{
  struct ImuFreqSrv
  {
    ImuFreqSrv() {}
    struct Request
    {
      int value;    //freq  max 1000
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      bool result; // true 成功， false 失败
      std::uint64_t timeStamp;
    } response;
  };
}