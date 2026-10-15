#include "control/l2/propulsion/Propulsion.h"

#include <algorithm>
#include <cmath>

namespace robofish::control::l2::propulsion {
namespace {
int approach(int previous, int target, float step) {
  // Integer nominal commands have 1% resolution. Floor the allowed step so
  // truncation toward zero cannot exceed the rate limit while decelerating.
  // ponytail: sub-1% steps hold; use a float nominal frame if finer ramps are needed.
  const int wholeStep = static_cast<int>(std::min(200.0f, std::floor(step)));
  return previous + std::clamp(target - previous, -wholeStep, wholeStep);
}
}  // namespace

Output allocate(const Request &request, const PropulsionRecipe &recipe,
                const ActuatorFrame &lastCommitted, uint32_t slewElapsedMs) {
  Output output;
  if (request.enabled && request.propulsionEnabled &&
      (request.mode == DriveMode::Prop || request.mode == DriveMode::Hybrid)) {
    const float drive = request.independentDrives ? request.propulsionDrivePercent : request.surgePercent;
    const float requestedCommon = drive * recipe.commandScale;
    const float common = std::clamp(requestedCommon, -100.0f, 100.0f);
    // Existing policy: preserve common drive, then use only remaining yaw range.
    const float requestedDifferential = request.yawPercent * recipe.yawGain;
    const float margin = 100.0f - std::abs(common);
    const float differential = std::clamp(requestedDifferential, -margin, margin);
    if (common != requestedCommon || differential != requestedDifferential)
      output.limits |= PropulsionLimited;
    output.targetLeft = static_cast<int>((common + differential) * request.outputScale);
    output.targetRight = static_cast<int>((common - differential) * request.outputScale);
  }
  const float step = recipe.ratePercentPerSecond * slewElapsedMs * 0.001f;
  output.left = request.immediate ? output.targetLeft :
      approach(lastCommitted.leftPropulsionPercent, output.targetLeft, step);
  output.right = request.immediate ? output.targetRight :
      approach(lastCommitted.rightPropulsionPercent, output.targetRight, step);
  if (output.left != output.targetLeft || output.right != output.targetRight)
    output.limits |= PropulsionRateLimited;
  output.common = (output.left + output.right) * 0.5f;
  output.differential = (output.left - output.right) * 0.5f;
  return output;
}

}  // namespace robofish::control::l2::propulsion
