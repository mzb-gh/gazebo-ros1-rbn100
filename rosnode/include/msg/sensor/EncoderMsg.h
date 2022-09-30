#pragma once

#include <cstdint>

namespace sensors_msgs
{
  //固定帧率上报
  struct EncoderMsg
  {
    uint32_t frameId;            //帧号
    std::uint64_t EncodeR;       //右边轮子编码器值
    std::uint64_t EncodeL;       //左边轮子编码器值
    std::uint64_t timeStamp;     // utc时间 单位 ms
  };
}
