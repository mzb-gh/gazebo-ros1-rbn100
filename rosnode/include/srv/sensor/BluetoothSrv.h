#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs
{

  struct BluetoothSrv
  {
    BluetoothSrv() {}

    struct Request
    {
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      std::uint64_t timeStamp;
      bool result; // true 连接成功， false 连接失败
    } response;
  };
}