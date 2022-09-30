#pragma once

#include <cstdint>

namespace sensors_msgs {

// 沿边传感器
struct WallSensorMsg {
  enum TOF_SIDE {
    LEFT = 0u,
    RIGHT = 1u,
  };
  uint32_t frameId;        // 帧号
  std::uint8_t sensorId;   // 哪一个tof
  std::uint16_t distance;  // 距离，单位mm
  std::uint64_t timeStamp;
};

}  // namespace sensors_msgs