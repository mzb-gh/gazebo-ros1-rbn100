#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs {
struct CabinControl {
  CabinControl() {}
  struct Request {
    std::uint64_t timeStamp;
    bool status;  //true  打开 false 关闭
  } request;

  struct Response {
    std::uint64_t timeStamp;
    bool result;   //true 成功 false 失败
  } response;
};
}  // namespace  sensor_srvs
