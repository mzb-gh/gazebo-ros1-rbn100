#pragma once

#include <cstdint>

namespace sensors_msgs {

struct ImuMsg {
  std::uint64_t sysTimeStamp;  // 收到数据的系统时间，只做参考
  std::uint64_t timeStamp;     // 经计算后的传感器时间ms,用于实际应用
  uint8_t sensorId;   // 传感器ID 
  uint32_t frameId;     // 数据帧号，标志第几帧数据
  /** IMU accelerometer data for 3-axis: X, Y, Z. */
  double accel[3];
  /** IMU gyroscope data for 3-axis: X, Y, Z. */
  double gyro[3];
  /** IMU timeStamp */
};

}  // namespace sensors_msgs
