#pragma once

#include <cstdint>

namespace sensors_msgs {
struct DropEvent {
  enum DropStatus {
    RELEASED = 0u,
    TRIGGER = 1u,
  };

  enum DropSide {
    LEFT = 0u,
    RIGHT = 1u,
    LEFT_RIGHT = 2u,
  };

  uint32_t frameId;   //帧号
  DropSide sensorId;  // which
  DropStatus status;  // 1:接触 0:弹开
  std::uint64_t timeStamp;
};
}  // namespace sensors_msgs