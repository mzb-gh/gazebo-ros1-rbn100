#pragma once

#include <cstdint>
namespace sensor_srvs
{

  struct MemorySrv
  {
    MemorySrv() {}

    struct Request
    {
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      float freeMem;   //剩余磁盘 单位： Gbyte
      float totalMem;  //总磁盘空间 单位： Gbyte
      float freeRam;   //剩余内存 单位： Gbyte
      float totalRam;  //总共内存 单位： Gbyte
      std::uint64_t timeStamp;
    } response;
  };

} // namespace sensor_srvs