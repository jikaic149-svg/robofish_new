#pragma once

#include "config/ControlConfig.h"
#include "control/l2/Types.h"

// One hardware profile for all sources. Legacy SDK virtual throttle/steering
// angles and 86-degree tail centers no longer select different physical gaits.
namespace robofish::config::motion {

inline robofish::control::l2::Geometry makeGeometry() {
  return {
      {lora::kTailFrontCenter, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE},
      {lora::kTailRearCenter, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE},
      {lora::kPectoralLeftCenter, lora::kPectoralLeftMinimum, lora::kPectoralLeftMaximum},
      {lora::kPectoralRightCenter, lora::kPectoralRightMinimum, lora::kPectoralRightMaximum}};
}

inline robofish::control::l2::Recipe makeRecipe(DriveMode mode) {
  robofish::control::l2::Recipe recipe;
  const bool hybrid = mode == DriveMode::Hybrid;
  recipe.tail.amplitudeDegrees = lora::kTailAmplitudeDegrees *
      (hybrid ? config::control::kHybridTailCommandScale : 1);
  recipe.tail.frequencyHz = lora::kTailFrequencyHz;
  recipe.tail.rearAmplitudeRatio = lora::kRearTailAmplitudeRatio;
  recipe.tail.phaseLagRad = (mode == DriveMode::Bionic ?
      lora::kBionicPhaseLagDegrees : lora::kHybridPhaseLagDegrees) *
      static_cast<float>(algorithm::kTwoPi / 360);
  // These are nominal yaw-to-tail geometric gains, not ESC wiring polarity.
  const float tailYawSign = lora::kTurnDirection * lora::kTailSteeringDirection;
  recipe.tail.turnBiasDegrees = lora::kTailTurnBiasDegrees * tailYawSign;
  recipe.tail.staticSteeringDegrees = lora::kTailStaticSteeringDegrees * tailYawSign;
  recipe.tail.rateDegreesPerSecond = lora::kServoStepPer10Ms * 100;
  recipe.propulsion.commandScale = hybrid ? config::control::kHybridPropulsionCommandScale : 1;
  // FRD: the left prop is at negative y, so positive (right) yaw needs T_L>T_R.
  // Do not reuse the old radio-specific steering sign for this geometric law.
  recipe.propulsion.yawGain = 1;
  recipe.propulsion.ratePercentPerSecond = config::control::kPropulsionRatePercentPerSecond;
  const auto offset = [](int center, int endpoint) {
    // Preserve the existing 132/48, 56/124 endpoint rounding.
    return static_cast<float>(static_cast<int>((endpoint - center) * lora::kPectoralScale));
  };
  recipe.pectoral.leftPositivePitchDegrees = offset(lora::kPectoralLeftCenter, lora::kPectoralLeftUp);
  recipe.pectoral.rightPositivePitchDegrees = offset(lora::kPectoralRightCenter, lora::kPectoralRightUp);
  recipe.pectoral.leftNegativePitchDegrees = offset(lora::kPectoralLeftCenter, lora::kPectoralLeftDown);
  recipe.pectoral.rightNegativePitchDegrees = offset(lora::kPectoralRightCenter, lora::kPectoralRightDown);
  // Roll geometry remains uncalibrated: its gains are explicitly zero.
  recipe.pectoral.leftYawDegrees = config::control::kPectoralLeftYawDegrees * lora::kTurnDirection;
  recipe.pectoral.rightYawDegrees = config::control::kPectoralRightYawDegrees * lora::kTurnDirection;
  recipe.pectoral.leftSurgeTrimDegrees = config::control::kPectoralLeftSurgeTrimDegrees;
  recipe.pectoral.rightSurgeTrimDegrees = config::control::kPectoralRightSurgeTrimDegrees;
  recipe.pectoral.leftTailTrimDegrees = config::control::kPectoralLeftTailTrimDegrees;
  recipe.pectoral.rightTailTrimDegrees = config::control::kPectoralRightTailTrimDegrees;
  recipe.pectoral.rateDegreesPerSecond = lora::kServoStepPer10Ms * 100;
  recipe.flowMinimumPercent = config::control::kFlowMinimumPercent;
  recipe.diveDragCompensationPercent = config::control::kDiveDragCompensationPercent;
  recipe.maximumSlewElapsedMs = config::control::kMaximumSlewElapsedMs;
  return recipe;
}

}  // namespace robofish::config::motion
