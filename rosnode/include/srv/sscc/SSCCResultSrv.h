#pragma once

#include <cstdint>
#include <string>

namespace sscc_srvs
{
  struct SSCCResultSrv
  {
    struct Request
    {
      std::uint64_t timeStamp;
    } request;
    struct Response
    {
      std::uint64_t timeStamp;
      bool result;
      cv::Mat K1, D1, R1, P1;
      cv::Mat K2, D2, R2, P2;
    } response;
  };
}
