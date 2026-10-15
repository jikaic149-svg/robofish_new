#include "control/l2/pectoral/Pectoral.h"

#include <algorithm>
#include <cmath>

#include "algorithm/PairedLimit.h"

namespace robofish::control::l2::pectoral {
namespace {
float approach(float previous, float target, float step) {
  return previous + std::clamp(target - previous, -step, step);
}
}  // namespace

Output allocate(const Request &request, const PectoralRecipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                float commonPropulsionFraction, float tailLoad, uint32_t slewElapsedMs) {
  Output output;
  float baseLeft = geometry.pectoralLeft.centerDegrees;
  float baseRight = geometry.pectoralRight.centerDegrees;
  float deltaLeft = 0;
  float deltaRight = 0;
  if (request.enabled) {
    const float pitchFraction = std::abs(request.pitchPercent) * 0.01f;
    baseLeft += pitchFraction * (request.pitchPercent >= 0 ?
        recipe.leftPositivePitchDegrees : recipe.leftNegativePitchDegrees);
    baseRight += pitchFraction * (request.pitchPercent >= 0 ?
        recipe.rightPositivePitchDegrees : recipe.rightNegativePitchDegrees);
    deltaLeft = request.rollPercent * 0.01f * recipe.leftRollDegrees +
        request.yawPercent * 0.01f * recipe.leftYawDegrees +
        commonPropulsionFraction * recipe.leftSurgeTrimDegrees + tailLoad * recipe.leftTailTrimDegrees;
    deltaRight = request.rollPercent * 0.01f * recipe.rightRollDegrees +
        request.yawPercent * 0.01f * recipe.rightYawDegrees +
        commonPropulsionFraction * recipe.rightSurgeTrimDegrees + tailLoad * recipe.rightTailTrimDegrees;
  }
  const float boundedLeft = std::clamp(baseLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees);
  const float boundedRight = std::clamp(baseRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  if (boundedLeft != baseLeft || boundedRight != baseRight) output.limits |= PectoralLimited;
  // Pitch is the primary target. Roll and optional calibrated feed-forward share
  // only its remaining travel; retain the pair's correction ratio at saturation.
  output.correctionScale = algorithm::pairedOffsetScale(boundedLeft, deltaLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees,
      boundedRight, deltaRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  output.targetLeft = std::clamp(boundedLeft + output.correctionScale * deltaLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees);
  output.targetRight = std::clamp(boundedRight + output.correctionScale * deltaRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  if (output.correctionScale < 1) output.limits |= PectoralLimited;
  const float step = recipe.rateDegreesPerSecond * slewElapsedMs * 0.001f;
  output.left = request.immediate ? output.targetLeft :
      approach(lastCommitted.pectoralLeftDegrees, output.targetLeft, step);
  output.right = request.immediate ? output.targetRight :
      approach(lastCommitted.pectoralRightDegrees, output.targetRight, step);
  if (output.left != output.targetLeft || output.right != output.targetRight)
    output.limits |= PectoralRateLimited;
  return output;
}

}  // namespace robofish::control::l2::pectoral
