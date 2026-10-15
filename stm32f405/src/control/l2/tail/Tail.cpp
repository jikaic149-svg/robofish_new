#include "control/l2/tail/Tail.h"

#include <algorithm>
#include <cmath>

namespace robofish::control::l2::tail {
namespace {
float approach(float previous, float target, float step) {
  return previous + std::clamp(target - previous, -step, step);
}
float boundedCenter(const JointGeometry &joint, float offset, uint32_t &limits) {
  const float requested = joint.centerDegrees + offset;
  const float result = std::clamp(requested, joint.minimumDegrees, joint.maximumDegrees);
  if (result != requested) limits |= TailLimited;
  return result;
}
}  // namespace

Output allocate(const Request &request, const TailRecipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                double phaseRad, uint32_t slewElapsedMs) {
  Output output;
  output.targetFront = geometry.tailFront.centerDegrees;
  output.targetRear = geometry.tailRear.centerDegrees;
  if (request.enabled && request.tailEnabled) {
    const bool oscillating = request.mode == DriveMode::Bionic || request.mode == DriveMode::Hybrid;
    const float yaw = request.independentTailYaw ? request.tailYawPercent : request.yawPercent;
    const float bias = yaw * 0.01f *
        (oscillating ? recipe.turnBiasDegrees : recipe.staticSteeringDegrees);
    const float frontCenter = boundedCenter(geometry.tailFront, bias, output.limits);
    const float rearCenter = boundedCenter(geometry.tailRear, bias, output.limits);
    output.targetFront = frontCenter;
    output.targetRear = rearCenter;
    if (oscillating) {
      float drive = request.independentDrives ? request.tailDrivePercent : request.surgePercent;
      if (!request.independentDrives && request.mode == DriveMode::Bionic && drive == 0)
        drive = std::abs(yaw);
      // Reverse propulsion is not obtained by multiplying both sine waves by -1.
      const float requestedAmplitude = std::max(0.0f, drive) * 0.01f *
          recipe.amplitudeDegrees * request.outputScale;
      float available = std::min(frontCenter - geometry.tailFront.minimumDegrees,
                                 geometry.tailFront.maximumDegrees - frontCenter);
      if (recipe.rearAmplitudeRatio != 0) {
        const float rearAvailable = std::min(rearCenter - geometry.tailRear.minimumDegrees,
            geometry.tailRear.maximumDegrees - rearCenter) / std::abs(recipe.rearAmplitudeRatio);
        available = std::min(available, rearAvailable);
      }
      const float amplitude = std::min(requestedAmplitude, std::max(0.0f, available));
      if (amplitude != requestedAmplitude) output.limits |= TailLimited;
      output.load = recipe.amplitudeDegrees > 0 ? amplitude / recipe.amplitudeDegrees : 0;
      output.targetFront += amplitude * static_cast<float>(std::sin(phaseRad));
      output.targetRear += amplitude * recipe.rearAmplitudeRatio *
          static_cast<float>(std::sin(phaseRad - recipe.phaseLagRad));
    }
  }
  // Floating-point roundoff must not leak an out-of-range endpoint to L3.
  output.targetFront = std::clamp(output.targetFront,
      geometry.tailFront.minimumDegrees, geometry.tailFront.maximumDegrees);
  output.targetRear = std::clamp(output.targetRear,
      geometry.tailRear.minimumDegrees, geometry.tailRear.maximumDegrees);
  const float step = recipe.rateDegreesPerSecond * slewElapsedMs * 0.001f;
  output.front = request.immediate ? output.targetFront :
      approach(lastCommitted.tailFrontDegrees, output.targetFront, step);
  output.rear = request.immediate ? output.targetRear :
      approach(lastCommitted.tailRearDegrees, output.targetRear, step);
  if (output.front != output.targetFront || output.rear != output.targetRear)
    output.limits |= TailRateLimited;
  return output;
}

}  // namespace robofish::control::l2::tail
