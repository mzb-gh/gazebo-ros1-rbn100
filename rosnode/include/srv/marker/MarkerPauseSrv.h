#pragma once

#include <cstdint>
#include <string>
#include "../../msg/common/error/ErrorMessage.h"
#include <opencv2/opencv.hpp>

namespace marker_srvs
{
  struct MarkerPauseSrv
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

