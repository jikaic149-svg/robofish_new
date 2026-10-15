#include "control/l0/IntentAdapter.h"

#include <algorithm>
#include <cmath>

#include "config/RuntimeConfig.h"

namespace robofish::control::l0 {
namespace {
float clippedPercent(float value) {
  // Preserve non-finite values for the L1 validator to reject explicitly.
  return std::isfinite(value) ? std::clamp(value, -100.0f, 100.0f) : value;
}
float power(uint8_t value) { return std::min<int>(value, 100); }
float pitchRequest(int legacyPitch) {
  if (legacyPitch == RF_PITCH_CENTER || legacyPitch == RF_PITCH_UI_NEUTRAL) return 0;
  legacyPitch = std::clamp(legacyPitch, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE);
  return legacyPitch < RF_PITCH_CENTER ?
      (RF_PITCH_CENTER - legacyPitch) * 100.0f / (RF_PITCH_CENTER - RF_PITCH_MIN_ANGLE) :
      -(legacyPitch - RF_PITCH_CENTER) * 100.0f / (RF_PITCH_MAX_ANGLE - RF_PITCH_CENTER);
}
}  // namespace

AdaptedIntent fromRemote(const BodyMotionTarget &target, bool immediate) {
  AdaptedIntent result;
  result.mode = target.mode;
  result.body.enabled = target.enabled;
  result.body.surgePercent = clippedPercent(target.surgePercent) *
      config::lora::kForwardDirection * (target.propulsionReversed ? -1 : 1);
  result.body.yawPercent = clippedPercent(target.yawPercent) * (target.steeringReversed ? -1 : 1);
  result.body.pitchPercent = clippedPercent(target.heavePercent);
  result.tailYawPercent = result.body.yawPercent * (target.tailReversed ? -1 : 1);
  const int knob = std::min<int>(target.knob, 1000);
  // Preserve the existing whole-percent gear quantization.
  result.outputScale = (config::lora::kGearMinimumPercent +
      knob * (config::lora::kGearMaximumPercent - config::lora::kGearMinimumPercent) / 1000) * 0.01f;
  result.allowFlowAssist = target.enabled && target.allowFlowAssist;
  result.immediate = immediate;
  return result;
}

AdaptedIntent fromManual(const ManualIntent &intent, int pitch, bool failsafe) {
  AdaptedIntent result;
  // Legacy idle still permits a standalone fin pitch request, without drive.
  result.mode = intent.mode == DriveMode::Idle ? DriveMode::Prop : intent.mode;
  result.body.enabled = true;
  result.body.pitchPercent = pitchRequest(pitch);
  result.independentDrives = true;
  result.tailEnabled = intent.tailMoving && intent.mode != DriveMode::Idle;
  result.propulsionEnabled = intent.propulsionMoving &&
      (intent.mode == DriveMode::Prop || intent.mode == DriveMode::Hybrid);
  const float tailSign = intent.reverse ? -1.0f : 1.0f;
  const float propSign = intent.reverse != intent.escReversed ? -1.0f : 1.0f;
  result.tailDrivePercent = result.tailEnabled &&
      (intent.mode == DriveMode::Bionic || intent.mode == DriveMode::Hybrid) ?
      power(intent.tailPowerPercent) * tailSign : 0;
  result.propulsionDrivePercent = result.propulsionEnabled ? power(intent.propulsionPowerPercent) * propSign : 0;
  result.body.surgePercent = intent.mode == DriveMode::Bionic ?
      result.tailDrivePercent : result.propulsionDrivePercent;
  const float yaw = intent.turning ? power(intent.turnStrengthPercent) *
      (intent.action == MotionAction::Left ? -1 : 1) : 0;
  result.body.yawPercent = yaw * (intent.steeringReversed ? -1 : 1);
  // This preference reverses tail steering geometry only. It never reverses
  // the oscillator's sine waves or claims to create reverse tail propulsion.
  result.tailYawPercent = yaw * (intent.tailReversed ? -1 : 1);
  if (failsafe) {
    result.mode = DriveMode::Prop;
    result.body = {0, 0, 100, 0, true};
    result.tailDrivePercent = result.propulsionDrivePercent = result.tailYawPercent = 0;
    result.tailEnabled = result.propulsionEnabled = false;
    result.immediate = true;
  }
  return result;
}

}  // namespace robofish::control::l0
