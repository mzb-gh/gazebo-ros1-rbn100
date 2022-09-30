#pragma once

#include <cstdint>
#include <string>

namespace sensor_srvs {
struct SensorControl {
  SensorControl() {}
  enum SENSOR { CLIFF = 1, BUMP, DROP };
  struct Request {
		SENSOR sensor;
		bool status;  //打开还是关闭
		uint64_t timsStamp;
  } request;

  struct Response {
		bool result;   //true  or  false
		uint64_t timsStamp;
  } response;
};
}  // namespace sensor_srvs

