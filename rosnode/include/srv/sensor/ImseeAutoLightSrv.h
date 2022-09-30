#pragma once

#include <cstdint>
#include <string>
// 纹理亮度控制
namespace sensor_srvs
{
  struct ImseeAutoLightSrv
  {
    ImseeAutoLightSrv() {}
    struct Request
    {
      bool autoLight;    //自动调亮度, true 使能， false 禁用
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      bool result; // true 成功 ， false 失败
      std::uint64_t timeStamp;
    } response;
  };
}