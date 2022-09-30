#pragma once

#include <cstdint>
#include <string>
#include "../../msg/common/error/ErrorMessage.h"


namespace marker_srvs
{
  struct MarkerStopSrv
  {
    struct Request
    {
      std::uint64_t timeStamp;
    } request;

    struct Response
    {
      std::uint64_t timeStamp;
      bool result;
      error_msgs::ErrorMessage errMsg;
    } response;
  };
}
