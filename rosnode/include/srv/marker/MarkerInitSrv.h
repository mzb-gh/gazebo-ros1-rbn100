#pragma once

#include <cstdint>
#include <string>
#include "../../msg/common/error/ErrorMessage.h"

namespace marker_srvs
{
  struct MarkerInitSrv
  {
    struct Request
    {
      std::uint64_t timeStamp;
      std::string configPath; 
    } request;

    struct Response
    {
      std::uint64_t timeStamp;
      bool result; // success: true, failure:false
      error_msgs::ErrorMessage errMsg;
    } response;
  };
};
