#pragma once

#include <vector>
#include <cstdint>

namespace sensors_msgs {

#define MAX_LASER_SIZE 2048

struct LaserScan {
  struct Header {
    std::uint32_t seq;
    std::uint64_t stamp;
  } header;
  float angle_min;
  float angle_max;
  float angle_increment;
  float time_increment;
  float scan_time;
  float range_min;
  float range_max;
  size_t dataSize = 0;                // 原始数据大小单位 1*float
  float ranges[MAX_LASER_SIZE];       // 根据 datasize 的大小取出对应的数据
  float intensities[MAX_LASER_SIZE];  // 根据 datasize 的大小取出对应的数据
};
}  // namespace sensors_msgs