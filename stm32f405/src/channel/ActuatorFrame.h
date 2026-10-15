#pragma once
#include <stdint.h>

#include "config/RuntimeConfig.h"

namespace robofish {

// 同一个控制周期的六路输出。角度/百分比分开，不能把 ESC 百分比当舵机角度。
struct ActuatorFrame {
  float tailFrontDegrees = config::lora::kTailFrontCenter;
  float tailRearDegrees = config::lora::kTailRearCenter;
  float pectoralLeftDegrees = config::lora::kPectoralLeftCenter;
  float pectoralRightDegrees = config::lora::kPectoralRightCenter;
  int leftPropulsionPercent = 0;
  int rightPropulsionPercent = 0;
  bool flowAssisted = false;
  // Fresh safe ADC voltage for the electrical output stage; zero forces neutral ESCs.
  uint16_t supplyMillivolts = 0;
};

}  // namespace robofish
