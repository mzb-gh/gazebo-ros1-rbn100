#pragma once

#include <cstdint>
#include <string>
#include "../../msg/common/error/ErrorMessage.h"
namespace sensor_srvs
{

  struct SensorInit
  {
    SensorInit() {}

    struct Request
    {
      std::string sensorConfigPath; //sensor config path
     
    } request;

    struct Response
    {
      bool result; // true 成功， false 失败
      std::uint64_t timeStamp;
      error_msgs::ErrorMessage errMsg;
    } response;
  };

} // namespace sensor_srvs