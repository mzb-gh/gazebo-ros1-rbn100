#pragma once

#include <cstdint>
#include <string>
#include "../../msg/common/error/ErrorMessage.h"

namespace marker_srvs
{
  struct MarkerStartSrv
  {
    struct Request
    {
      std::uint64_t timeStamp;
      int status ; // 0: start, 1:end
    } request;
    struct Response
    {
      std::uint64_t timeStamp;
      bool result; //success:true, failure:false 
      error_msgs::ErrorMessage errMsg;
    } response;
  };
}
