#pragma once

#include <iostream>
#include <map>

namespace sensor_srvs {
struct SelfTest {
  enum SelfTestResult {
    ALLSUCCESS = 0,
    IMUFAILED,
    IMGFAILED,
    TOFFAILED,
    LASERFAILED,
    ULTRAFAILED,
    CLIFFFAILED,
    ENCODERFAILED,
    VELOCITYFAILED,
    ALL  // 无意义，标志传感器数量
  };
  struct Request {
    std::uint64_t timeStamp;
  } request;
  struct Response {
    uint8_t selfTest[ALL]; //失败的时候，会把数组对应的字节置一
    std::uint64_t timeStamp;
  } response;
};

}  // namespace sensor_srvs