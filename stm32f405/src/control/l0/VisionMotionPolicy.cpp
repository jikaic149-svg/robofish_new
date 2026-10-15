#include "control/l0/VisionMotionPolicy.h"

#include <algorithm>
#include <cmath>

#include "config/RuntimeConfig.h"

namespace robofish::control::l0 {

VisionMotionDemand mapVisionDemand(float leftSteeringPercent, float powerPercent,
                                  float curveSeverity) {
  if (!std::isfinite(leftSteeringPercent) || !std::isfinite(powerPercent) ||
      !std::isfinite(curveSeverity)) return {};
  const int steering = static_cast<int>(
      std::lround(std::clamp(leftSteeringPercent, -100.0f, 100.0f)));
  const int requestedPower = static_cast<int>(
      std::lround(std::clamp(powerPercent, 0.0f, 100.0f)));
  const float turnDemand = std::max(
      std::clamp(curveSeverity, 0.0f, 1.0f), static_cast<float>(std::abs(steering)) / 100.0f);

  const int tailPower = static_cast<int>(std::lround(
      RF_VISION_TAIL_STRAIGHT_POWER_PERCENT +
      (RF_VISION_TAIL_TURN_POWER_PERCENT -
       RF_VISION_TAIL_STRAIGHT_POWER_PERCENT) *
          turnDemand));
  const int propulsionLimit = static_cast<int>(std::lround(
      100.0f - (100.0f - RF_VISION_PROP_CURVE_MIN_POWER_PERCENT) *
                   turnDemand));

  VisionMotionDemand demand;
  demand.requestedPowerPercent = static_cast<uint8_t>(requestedPower);
  demand.turnStrengthPercent =
      static_cast<uint8_t>(std::abs(steering));
  demand.tailPowerPercent = static_cast<uint8_t>(
      requestedPower > 0 ? std::clamp(tailPower, 0, 100) : 0);
  demand.propulsionPowerPercent = static_cast<uint8_t>(
      std::clamp(std::min(requestedPower, propulsionLimit), 0, 100));

  if (requestedPower == 0) {
    demand.action = MotionAction::Stopped;
  } else if (steering > 0) {
    demand.action = MotionAction::Left;
  } else if (steering < 0) {
    demand.action = MotionAction::Right;
  } else {
    demand.action = MotionAction::Forward;
  }
  return demand;
}

}  // namespace robofish::control::l0
