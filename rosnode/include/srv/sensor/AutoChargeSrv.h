#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs {
struct AutoCharge {
  AutoCharge() {}
  struct Request {
    std::uint64_t timeStamp;
    bool isAutoCharge;  // false 手动充电，true 自动充电
  } request;

  struct Response {
    std::uint64_t timeStamp;
    bool result; 
  } response;
};
}  // namespace  sensor_srvs