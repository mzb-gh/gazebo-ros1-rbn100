#pragma once

#include <cstdint>
#include <string>

namespace sensors_msgs {

struct BluetoothMsg
{
  std::uint8_t connectStatus; //连接状态
  std::uint8_t ledStatus;     // 指示灯状态
  std::uint64_t timeStamp;
};

}