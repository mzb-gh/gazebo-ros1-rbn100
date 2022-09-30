#pragma once

#include <cstdint>
#include <string>

namespace sscc_srvs
{
  struct SSCCPauseSrv
  {
    struct Request
    {
      std::uint64_t timeStamp;
    } request;
    struct Response
    {
      std::uint64_t timeStamp;
      bool result;
    } response;
  };
}
