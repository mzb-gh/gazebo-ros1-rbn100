#pragma once
#include <cstdint>

namespace sensors_msgs {
struct UltraData {
  uint8_t frameId; // 帧号
  float distance[10];  // 超声波 距离值，单位 cm
  std::uint64_t time;
};
}  // namespace sensors_msgs