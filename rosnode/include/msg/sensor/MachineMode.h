#pragma once

#include <cstdint>

namespace sensors_msgs {

struct MachineMode {
	enum MACHINE_MODE_NUM
	{
		NULL_0 = 0,
		MACHINE_SLEEP,
		MACHINE_WAIT,
		MACHINE_DC_CHARGE,
		MACHINE_AUTO_CHARGE,
		MACHINE_RUN,
		MACHINE_TEST,
		MACHINE_WIFI_SLEEP
	};
	
	uint8_t machineModeData;
	std::uint64_t timeStamp;
};

}