#pragma once

#include <cstdint>

namespace sensors_msgs
{

struct BumperEvent {
  enum BumperStatus {
    RELEASED = 0u,
    TRIGGER = 1u,
  };

  enum BumperSide {
    LEFT = 0u,
    RIGHT = 1u,
    LEFT_RIGHT = 2u,  //左右两侧同时触发
  };
  uint32_t frameId;     //帧号
  BumperSide sensorId;  // which bumper
  BumperStatus status;  // 1:接触 0:弹开
  std::uint64_t timeStamp;
};
}  // namespace sensors_msgs