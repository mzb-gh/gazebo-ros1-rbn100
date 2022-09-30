#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs
{

  struct SensorDeinit
  {
    SensorDeinit() {}

    struct Request
    {
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      bool result; // true 成功， false 失败
      std::uint64_t timeStamp;
    } response;
  };

} // namespace sensor_srvs