#pragma once

#include <cstdint>

namespace sensors_msgs {

struct CliffEvent {
  enum CliffStatus {
    RELEASED = 0u,
    TRIGGER = 1u,
  };

  enum CliffSide {
    LEFT = 0u,
    RIGHT = 1u,
    LEFT_RIGHT = 2u,  //左右两侧同时触发
  };
  uint32_t frameId;    // 帧号
  CliffSide sensorId;  // 哪个cliff触发
  CliffStatus status;  // cliff 的状态
  std::uint64_t timeStamp;
};
}