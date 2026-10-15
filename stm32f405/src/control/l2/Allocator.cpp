#include "control/l2/Allocator.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>

#include "control/l2/pectoral/Pectoral.h"
#include "control/l2/propulsion/Propulsion.h"
#include "control/l2/tail/Tail.h"

namespace robofish::control::l2 {
namespace {
bool finiteValues(std::initializer_list<float> values) {
  for (const float value : values) if (!std::isfinite(value)) return false;
  return true;
}
bool validJoint(const JointGeometry &joint) {
  return finiteValues({joint.centerDegrees, joint.minimumDegrees, joint.maximumDegrees}) &&
      joint.minimumDegrees <= joint.centerDegrees && joint.centerDegrees <= joint.maximumDegrees;
}
bool validGeometry(const Geometry &geometry) {
  return validJoint(geometry.tailFront) && validJoint(geometry.tailRear) &&
      validJoint(geometry.pectoralLeft) && validJoint(geometry.pectoralRight);
}
bool validRecipe(const Recipe &recipe) {
  const auto &tail = recipe.tail;
  const auto &prop = recipe.propulsion;
  const auto &fin = recipe.pectoral;
  return finiteValues({tail.amplitudeDegrees, tail.frequencyHz, tail.rearAmplitudeRatio,
      tail.phaseLagRad, tail.turnBiasDegrees, tail.staticSteeringDegrees, tail.rateDegreesPerSecond,
      prop.commandScale, prop.yawGain, prop.ratePercentPerSecond,
      fin.leftPositivePitchDegrees, fin.rightPositivePitchDegrees,
      fin.leftNegativePitchDegrees, fin.rightNegativePitchDegrees,
      fin.leftRollDegrees, fin.rightRollDegrees, fin.leftYawDegrees, fin.rightYawDegrees,
      fin.leftSurgeTrimDegrees, fin.rightSurgeTrimDegrees,
      fin.leftTailTrimDegrees, fin.rightTailTrimDegrees, fin.rateDegreesPerSecond,
      recipe.flowMinimumPercent, recipe.diveDragCompensationPercent}) &&
      tail.amplitudeDegrees >= 0 && tail.frequencyHz >= 0 && tail.rateDegreesPerSecond >= 0 &&
      prop.commandScale >= 0 && prop.ratePercentPerSecond >= 0 && fin.rateDegreesPerSecond >= 0 &&
      recipe.flowMinimumPercent >= 0 && recipe.flowMinimumPercent <= 100 &&
      recipe.diveDragCompensationPercent >= 0;
}
bool validRequest(const Request &request) {
  return finiteValues({request.surgePercent, request.yawPercent, request.pitchPercent,
      request.rollPercent, request.tailDrivePercent, request.propulsionDrivePercent,
      request.tailYawPercent, request.outputScale}) && request.outputScale >= 0 && request.outputScale <= 1 &&
      (request.mode == DriveMode::Idle || request.mode == DriveMode::Bionic ||
       request.mode == DriveMode::Prop || request.mode == DriveMode::Hybrid);
}
bool validHistory(const ActuatorFrame &frame, const Geometry &geometry) {
  const auto within = [](float value, const JointGeometry &joint) {
    return std::isfinite(value) && value >= joint.minimumDegrees && value <= joint.maximumDegrees;
  };
  return within(frame.tailFrontDegrees, geometry.tailFront) &&
      within(frame.tailRearDegrees, geometry.tailRear) &&
      within(frame.pectoralLeftDegrees, geometry.pectoralLeft) &&
      within(frame.pectoralRightDegrees, geometry.pectoralRight) &&
      frame.leftPropulsionPercent >= -100 && frame.leftPropulsionPercent <= 100 &&
      frame.rightPropulsionPercent >= -100 && frame.rightPropulsionPercent <= 100;
}
ActuatorFrame neutralFrame(const Geometry &geometry) {
  ActuatorFrame frame;
  frame.tailFrontDegrees = geometry.tailFront.centerDegrees;
  frame.tailRearDegrees = geometry.tailRear.centerDegrees;
  frame.pectoralLeftDegrees = geometry.pectoralLeft.centerDegrees;
  frame.pectoralRightDegrees = geometry.pectoralRight.centerDegrees;
  return frame;
}
float percent(float value) { return std::clamp(value, -100.0f, 100.0f); }
}  // namespace

Result allocate(const Request &input, const Recipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                State &state, uint32_t nowMs, uint32_t elapsedMs) {
  Result result;
  const bool geometryValid = validGeometry(geometry);
  if (!geometryValid || !validRecipe(recipe) || !validRequest(input) ||
      !validHistory(lastCommitted, geometry) ||
      (state.gait.initialized && !std::isfinite(state.gait.radians))) {
    // Invalid data is explicit and never reaches sin(), clamp(lo>hi), or L3 as NaN.
    // Invalid geometry has no usable neutral; return the existing frame defaults.
    result.targetFrame = geometryValid ? neutralFrame(geometry) : ActuatorFrame{};
    result.candidateFrame = result.targetFrame;
    result.limitedReasons = InvalidInput;
    result.phaseRad = std::isfinite(state.gait.radians) ? state.gait.radians : 0;
    return result;
  }

  Request request = input;
  request.surgePercent = percent(request.surgePercent);
  request.yawPercent = percent(request.yawPercent);
  request.pitchPercent = percent(request.pitchPercent);
  request.rollPercent = percent(request.rollPercent);
  request.tailDrivePercent = percent(request.tailDrivePercent);
  request.propulsionDrivePercent = percent(request.propulsionDrivePercent);
  request.tailYawPercent = percent(request.tailYawPercent);
  if (request.mode == DriveMode::Idle) request.enabled = false;

  const bool tailDriveActive = request.tailEnabled &&
      (request.mode == DriveMode::Bionic || request.mode == DriveMode::Hybrid);
  const bool propDriveActive = request.propulsionEnabled &&
      (request.mode == DriveMode::Prop || request.mode == DriveMode::Hybrid);
  const float requestedTail = tailDriveActive ?
      (request.independentDrives ? request.tailDrivePercent : request.surgePercent) : 0;
  const float requestedProp = propDriveActive ?
      (request.independentDrives ? request.propulsionDrivePercent : request.surgePercent) : 0;
  // Only drives that this mode will actually execute establish its direction.
  // A disabled/stale prop request cannot turn a pure-tail forward action into
  // an unsupported reverse, nor can Prop's unused tail drive disable its rudder.
  const bool backward = requestedTail < 0 || requestedProp < 0;
  if (request.enabled && request.mode == DriveMode::Bionic && backward) {
    // Reject the whole unsupported pure-tail action. No old horizontal request,
    // yaw-driven oscillation, pitch request, or flow assist survives this request.
    request.enabled = false;
    request.allowFlowAssist = false;
    result.unsupportedReverse = true;
    result.limitedReasons |= ReverseUnsupported;
  } else if (request.enabled && backward) {
    // Prop/Hybrid reverse is provided by the reversible props. A static tail
    // steering law in reversed flow has not been calibrated for this machine.
    request.tailEnabled = false;
    request.allowFlowAssist = false;
  }
  if (!request.enabled) {
    request.allowFlowAssist = false;
    request.surgePercent = request.yawPercent = 0;
    request.pitchPercent = request.rollPercent = 0;
    request.tailDrivePercent = request.propulsionDrivePercent = 0;
    request.tailYawPercent = 0;
  }

  bool flowAssisted = false;
  if (request.enabled && !backward) {
    const float minimum = request.allowFlowAssist && request.pitchPercent != 0 ?
        std::abs(request.pitchPercent) * recipe.flowMinimumPercent * 0.01f : 0;
    const float drag = std::abs(request.pitchPercent) * recipe.diveDragCompensationPercent * 0.01f;
    const auto assistedDrive = [&](float drive, bool active) {
      if (!active) return drive;
      if (drive < minimum) {
        drive = minimum;
        flowAssisted = true;
      }
      return drive > 0 ? std::min(100.0f, drive + drag) : drive;
    };
    if (request.independentDrives) {
      // Each active node retains its own upstream contribution. In particular,
      // body.surge is descriptive here and must not overwrite a 20/80 split.
      request.tailDrivePercent = assistedDrive(request.tailDrivePercent, tailDriveActive);
      request.propulsionDrivePercent = assistedDrive(request.propulsionDrivePercent, propDriveActive);
    } else {
      request.surgePercent = assistedDrive(request.surgePercent, tailDriveActive || propDriveActive);
    }
  }

  // One parent-owned clock for both joints, including uint32 tick wraparound.
  result.phaseRad = algorithm::advanceGait(state.gait, nowMs, recipe.tail.frequencyHz);
  const uint32_t slewMs = std::min(elapsedMs, recipe.maximumSlewElapsedMs);
  const auto tail = tail::allocate(request, recipe.tail, geometry, lastCommitted, result.phaseRad, slewMs);
  const auto propulsion = propulsion::allocate(request, recipe.propulsion, lastCommitted, slewMs);
  const auto pectoral = pectoral::allocate(request, recipe.pectoral, geometry, lastCommitted,
      (propulsion.targetLeft + propulsion.targetRight) * 0.005f, tail.load, slewMs);

  result.targetFrame = neutralFrame(geometry);
  result.targetFrame.tailFrontDegrees = tail.targetFront;
  result.targetFrame.tailRearDegrees = tail.targetRear;
  result.targetFrame.pectoralLeftDegrees = pectoral.targetLeft;
  result.targetFrame.pectoralRightDegrees = pectoral.targetRight;
  result.targetFrame.leftPropulsionPercent = propulsion.targetLeft;
  result.targetFrame.rightPropulsionPercent = propulsion.targetRight;
  result.targetFrame.flowAssisted = flowAssisted;
  result.candidateFrame = result.targetFrame;
  result.candidateFrame.tailFrontDegrees = tail.front;
  result.candidateFrame.tailRearDegrees = tail.rear;
  result.candidateFrame.pectoralLeftDegrees = pectoral.left;
  result.candidateFrame.pectoralRightDegrees = pectoral.right;
  result.candidateFrame.leftPropulsionPercent = propulsion.left;
  result.candidateFrame.rightPropulsionPercent = propulsion.right;
  result.commonPropulsionPercent = propulsion.common;
  result.differentialPropulsionPercent = propulsion.differential;
  result.tailLoad = tail.load;
  result.pectoralCorrectionScale = pectoral.correctionScale;
  result.limitedReasons |= tail.limits | propulsion.limits | pectoral.limits;
  // Finite individual coefficients can still overflow when added together.
  // Keep that failure explicit rather than offering a non-finite PWM target.
  if (!validHistory(result.targetFrame, geometry) || !validHistory(result.candidateFrame, geometry)) {
    result.targetFrame = result.candidateFrame = neutralFrame(geometry);
    result.commonPropulsionPercent = result.differentialPropulsionPercent = result.tailLoad = 0;
    result.pectoralCorrectionScale = 1;
    result.limitedReasons |= InvalidInput;
  }
  return result;
}

}  // namespace robofish::control::l2
