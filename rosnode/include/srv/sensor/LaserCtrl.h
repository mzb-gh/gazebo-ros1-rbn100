#pragma once

#include <cstdint>
#include <string>

//补光亮度控制
namespace sensor_srvs {
struct LaserCtrl {
  enum LaserCmdType {
    FREQ = 1,
    FILTER,
  };
  LaserCtrl() {}
  struct Request {
    LaserCmdType Type;
    union CMD {    // 不支持同时配置，只能一条一条发送
      int freq;    //频率配置
      int filter;  //滤波器配置
    } config;
    std::uint64_t timeStamp;
  } request;

  struct Response {
    bool result;  // true 连接成功， false 连接失败
    std::uint64_t timeStamp;
  } response;
};
}  // namespace sensor_srvs