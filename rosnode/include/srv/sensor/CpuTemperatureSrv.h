#pragma once

#include <cstdint>
namespace sensor_srvs
{

  struct CpuTemperatureSrv
  {
    CpuTemperatureSrv() {}

    struct Request
    {
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      float temperature; //总共内存 单位： 摄氏度
      std::uint64_t timeStamp;
    } response;
  };

} // namespace sensor_srvs