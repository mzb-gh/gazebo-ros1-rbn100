#pragma once

#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


namespace sensors_msgs {

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 400
//双目数据与纹理灯控制
class ImgMsg {
 public:
  enum TX_IR_STATE {
    TX_IR_OFF = 0u,  // 0：纹理和补光都不亮
    TX_ON = 1u,      // 1：纹理亮
    IR_ON = 2u,      // 2：补光亮
    TX_IR_ON = 3u    // 3：纹理补光同时亮
  };

  //纹理灯控制
  struct TxIrCtrol {
    int txIrState;      // 1、3纹理亮， 0、2非纹理亮
    int irLightValue;   //补光灯亮度值 0～100
    int txLightValue;   //纹理亮度值 0～100
    int envLightValue;  //环境光亮度 单位lux
  };
  std::uint64_t sysTimeStamp;  // 收到数据的系统时间，只做参考
  std::uint64_t timeStamp;     // 经计算后的传感器时间ms,用于实际应用
  int imseeId;                 // 模组ID号，非模组内部ID，类似于handle
  int cameraWidth;             // 图像宽度（width x 2）
  int cameraHeight;            // 图像高度
  int cameraFps;               // 图像帧率
  int cameraIndex;             // 图像当前帧号//原始值
  int imgSize;                 // 图像大小
  int channel;                 // 颜色通道，常数1
  float timeExpose;            // 曝光时间(ms)
  uint8_t left[256000];        // 640x400 bytes      cv::Mat 会引用 此指针
  uint8_t right[256000];       // 640x400 bytes      cv::Mat 会引用 此指针
  struct TxIrCtrol txIrStatus;  // 纹理灯控制
};
}
 
