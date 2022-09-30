#pragma once

#include <vector>
#include <string>



namespace sensors_msgs {

struct PointCloud2 {
  struct Header {
    std::uint32_t seq;
    std::uint64_t stamp;
  } header;
  std::uint32_t height;
  std::uint32_t width;
  struct PointField {
    std::string name;
    std::uint32_t offset;
    enum DataType {
      INT8 = 1,
      UINT8 = 2,
      INT16 = 3,
      UINT16 = 4,
      INT32 = 5,
      UINT32 = 6,
      FLOAT32 = 7,
      FLOAT64 = 8
    };
    std::uint8_t datatype;
    std::uint32_t count;
  };
  std::vector<PointField> fields;
  bool is_bigendian;
  std::uint32_t point_step;
  std::uint32_t row_step;
  std::vector<std::uint8_t> data;
  bool is_dense;
};

}
