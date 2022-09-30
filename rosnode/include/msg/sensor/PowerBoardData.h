#pragma once

#include <cstdint>
#include <string>

namespace sensors_msgs {

struct BSMMessage{
  float battery; //电池电量百分比
  float batteryVlotage; //电池电压
  float batteryCurrent; //电池电流
};

struct PowerBoardMsg
{
  uint64_t timeStamp;
  int batteryVlotage ; //电池电压 mv
  int navigationBoardCurrent;//导航版电流 ma
  int bldcCurrent;//bldc 电流 ma
  int16_t voltage12V;//12V电压 mv
  int16_t current12V;//12V电流 ma
  int16_t voltage5V;//5v电压 mv
  int16_t currebt5V;//5V电流 ma
  uint8_t chargeStatus;//充电状态 0：没有充电 1：线充 2：充电桩
  int chargeVoltage;//充电电压 mv
  int chargeCurrent;//充电电流 ma
  uint32_t warningStatus;//告警状态
  uint8_t infraredTransmittingId;//红外发射管id
  uint8_t infraredRecvingId;//红外接收管id
  uint8_t chargePileId;//充电桩ID
  uint8_t chargePileStatus;//充电桩状态
  BSMMessage message;
};
}



