#pragma once

#include <stdint.h>

#include "channel/MotionTypes.h"

namespace robofish::control::l0 {

struct VisionMotionDemand {
  MotionAction action = MotionAction::Stopped;
  uint8_t turnStrengthPercent = 0;
  uint8_t requestedPowerPercent = 0;
  uint8_t tailPowerPercent = 0;
  uint8_t propulsionPowerPercent = 0;
};

// PC path guidance uses left-positive heading error (FLU/image heading).
// This L0 boundary converts it to MotionAction; L1 yaw is right-positive.
VisionMotionDemand mapVisionDemand(float leftSteeringPercent, float powerPercent,
                                  float curveSeverity);

}  // namespace robofish::control::l0
