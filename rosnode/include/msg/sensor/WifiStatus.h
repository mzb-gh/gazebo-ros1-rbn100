#pragma once

#include <cstdint>

namespace sensors_msgs {
//具体内容待定
struct WifiStatus {
  std::uint8_t connectStatus;  // wifi连接状态 0： 没有 非0： 就是以连接
  std::uint8_t wifiStrength;      // wifi信号强度 0～100
  std::uint64_t timeStamp;  
};

}  // namespace sensors_msgs