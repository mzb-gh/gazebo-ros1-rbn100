#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs {
struct SetLedSrv {
  SetLedSrv() {}
  struct Request {
    enum LedColor {
      WHITE = 0,
      RED,
      BLUE,
      GREEN,
      YELLOW,
    };
    enum Direction {
      CLOSE = 0,      // 全部关闭
      LEFTFRONT = 1,  // 左前方灯
      RIGHTFRONT,     //右前方灯
      FORWARD,        //前面两个灯
      LEFTBACK,       //左后方灯
      LEFT,           //左侧两个灯
      RIGHT,          //右侧两个灯
      BACK,           // 后面两个灯
      RIGHTBACK,      //右后面
      ALLON           //所有的灯全打开
    };
    uint64_t timeStamp;
    LedColor color;  //设置灯的颜色
    Direction dir;   //设置哪个灯亮
    bool isBling;    //设置是否闪烁
  } request;
  struct Response {
    bool result;  //设置的结果
    uint64_t timeStamp;
  } response;
};

}  // namespace sensor_srvs
