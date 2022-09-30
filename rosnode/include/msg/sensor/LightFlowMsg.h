#pragma once

#include <cstdint>

namespace sensors_msgs {

struct LightFlowMsg {
	uint32_t frameId; // 帧号
	int flowX;		   // x轴方向数据
	int flowY;		   // y轴方向数据
	std::uint64_t timeStamp;
};

}
