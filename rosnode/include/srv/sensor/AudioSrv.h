#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs {
struct AudioSrv {
  AudioSrv() {}
  struct Request {
    std::uint64_t timeStamp;
    std::string MusicFile; //传入的音乐文件
    int voice;  //需要设置的音量
  } request;

  struct Response {
    std::uint64_t timeStamp;
		int voice;             //当前音量
    bool result; 
  } response;
};
}  // namespace  sensor_srvs
