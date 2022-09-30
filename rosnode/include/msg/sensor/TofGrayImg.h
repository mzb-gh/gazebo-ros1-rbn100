#pragma once

#include <cstdint>



namespace sensors_msgs {
  
struct TofGrayImg {
  int tofID;    //id号
  uint8_t sn[8];    //序列号，serial number
  int width;    //宽 224
  int height;   //高 172
  uint64_t frameIndex; //帧号
  std::uint64_t time_stamp;  //时间戳
  uint8_t data[224 * 172];

};

}  // namespace sensors_msgs