#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "channel/FrameApplyResult.h"
#include "control/l2/Allocator.h"

namespace allocation = robofish::control::l2;
namespace {
constexpr double kPi = 3.14159265358979323846;
bool near(float left, float right) { return std::abs(left - right) < 0.001f; }
allocation::Geometry geometry() {
  return {{90, 20, 152}, {90, 20, 152}, {90, 45, 145}, {90, 35, 155}};
}
allocation::Recipe recipe() {
  allocation::Recipe value;
  value.tail = {25, 1.5f, 0.9f, static_cast<float>(72 * kPi / 180), 18, 40, 250};
  value.propulsion = {1, 1, 500};
  value.pectoral.leftPositivePitchDegrees = 42;
  value.pectoral.rightPositivePitchDegrees = -42;
  value.pectoral.leftNegativePitchDegrees = -34;
  value.pectoral.rightNegativePitchDegrees = 34;
  value.pectoral.rateDegreesPerSecond = 250;
  value.maximumSlewElapsedMs = 20;
  value.flowMinimumPercent = 100;
  return value;
}
allocation::Request request() {
  allocation::Request value;
  value.mode = DriveMode::Hybrid;
  value.enabled = true;
  value.immediate = true;
  return value;
}
allocation::State phase(double radians) {
  allocation::State value;
  value.gait = {radians, 0, true};
  return value;
}
void sixChannelCompositionAndSlew() {
  auto input = request();
  input.surgePercent = 60;
  input.yawPercent = 20;
  input.pitchPercent = 20;
  auto state = phase(kPi / 2);
  const robofish::ActuatorFrame history;
  auto result = allocation::allocate(input, recipe(), geometry(), history, state, 0, 20);
  assert(near(result.targetFrame.tailFrontDegrees, 108.6f));
  assert(near(result.targetFrame.tailRearDegrees, 97.77173f));
  assert(near(result.targetFrame.pectoralLeftDegrees, 98.4f));
  assert(near(result.targetFrame.pectoralRightDegrees, 81.6f));
  assert(result.targetFrame.leftPropulsionPercent == 80);
  assert(result.targetFrame.rightPropulsionPercent == 40);
  assert(result.limitedReasons == 0);
  input.immediate = false;
  result = allocation::allocate(input, recipe(), geometry(), history, state, 0, 20);
  assert(near(result.candidateFrame.tailFrontDegrees, 95));
  assert(near(result.candidateFrame.tailRearDegrees, 95));
  assert(near(result.candidateFrame.pectoralLeftDegrees, 95));
  assert(near(result.candidateFrame.pectoralRightDegrees, 85));
  assert(result.candidateFrame.leftPropulsionPercent == 10);
  assert(result.candidateFrame.rightPropulsionPercent == 10);
  assert(result.limitedReasons == (allocation::TailRateLimited |
      allocation::PropulsionRateLimited | allocation::PectoralRateLimited));
  assert(history.leftPropulsionPercent == 0 && history.tailFrontDegrees == 90);
}
void independentContributionsAndCommonPriority() {
  auto input = request();
  input.independentDrives = true;
  input.tailDrivePercent = 80;
  input.propulsionDrivePercent = 20;
  auto state = phase(kPi / 2);
  const auto result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.targetFrame.tailFrontDegrees, 110));
  assert(result.targetFrame.leftPropulsionPercent == 20);
  assert(result.targetFrame.rightPropulsionPercent == 20);
  input.propulsionDrivePercent = 80;
  input.yawPercent = 40;
  const auto saturated = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(saturated.targetFrame.leftPropulsionPercent == 100);
  assert(saturated.targetFrame.rightPropulsionPercent == 60);
  assert(saturated.limitedReasons & allocation::PropulsionLimited);
  input.mode = DriveMode::Prop;
  input.yawPercent = 20;
  input.independentTailYaw = true;
  input.tailYawPercent = -20;
  const auto separateYaw = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(separateYaw.targetFrame.tailFrontDegrees, 82));
  assert(separateYaw.targetFrame.leftPropulsionPercent == 100);
  assert(separateYaw.targetFrame.rightPropulsionPercent == 60);
}
void reverseIsNotPhaseInversion() {
  auto input = request();
  input.mode = DriveMode::Bionic;
  input.surgePercent = -60;
  input.yawPercent = 40;
  input.pitchPercent = 100;
  input.rollPercent = 100;
  input.allowFlowAssist = true;
  auto state = phase(kPi / 2);
  const auto rejected = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(rejected.unsupportedReverse);
  assert(rejected.limitedReasons & allocation::ReverseUnsupported);
  assert(near(rejected.targetFrame.tailFrontDegrees, 90));
  assert(near(rejected.targetFrame.tailRearDegrees, 90));
  assert(near(rejected.targetFrame.pectoralLeftDegrees, 90));
  assert(near(rejected.targetFrame.pectoralRightDegrees, 90));
  assert(rejected.targetFrame.leftPropulsionPercent == 0);
  assert(!rejected.targetFrame.flowAssisted);
  input.mode = DriveMode::Hybrid;
  input.pitchPercent = 0;
  const auto hybrid = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(!hybrid.unsupportedReverse);
  assert(near(hybrid.targetFrame.tailFrontDegrees, 90));
  assert(near(hybrid.targetFrame.tailRearDegrees, 90));
  assert(hybrid.targetFrame.leftPropulsionPercent == -20);
  assert(hybrid.targetFrame.rightPropulsionPercent == -100);
  assert(!hybrid.targetFrame.flowAssisted);
}
void unusedDrivesCannotChangeTheExecutedDirection() {
  auto input = request();
  input.mode = DriveMode::Bionic;
  input.independentDrives = true;
  input.tailDrivePercent = 80;
  input.propulsionDrivePercent = -80;  // Mode cannot use this node.
  auto state = phase(kPi / 2);
  auto result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(!result.unsupportedReverse);
  assert(near(result.targetFrame.tailFrontDegrees, 110));
  assert(result.targetFrame.leftPropulsionPercent == 0);
  input.mode = DriveMode::Hybrid;
  input.propulsionEnabled = false;
  result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.targetFrame.tailFrontDegrees, 110));
  assert(result.targetFrame.leftPropulsionPercent == 0);
  input.mode = DriveMode::Prop;
  input.tailDrivePercent = -80;  // Static tail rudder ignores this drive field.
  input.propulsionEnabled = true;
  input.propulsionDrivePercent = 60;
  input.yawPercent = 20;
  result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.targetFrame.tailFrontDegrees, 98));
  assert(result.targetFrame.leftPropulsionPercent == 80);
  assert(result.targetFrame.rightPropulsionPercent == 40);
  input.mode = DriveMode::Bionic;
  input.tailEnabled = false;
  result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(!result.unsupportedReverse);
  assert(near(result.targetFrame.tailFrontDegrees, 90));
}
void tailGeometryAndPhaseAreShared() {
  auto input = request();
  input.surgePercent = 100;
  auto configuration = recipe();
  configuration.tail.rearAmplitudeRatio = 0.5f;
  configuration.tail.phaseLagRad = 0;
  auto limits = geometry();
  limits.tailRear = {90, 85, 95};
  auto state = phase(kPi / 2);
  const auto constrained = allocation::allocate(input, configuration, limits, {}, state, 0, 20);
  assert(near(constrained.targetFrame.tailFrontDegrees, 100));
  assert(near(constrained.targetFrame.tailRearDegrees, 95));
  assert(constrained.limitedReasons & allocation::TailLimited);
  assert(near(constrained.tailLoad, 0.4f));

  configuration.tail.frequencyHz = 1;
  state = phase(0);
  state.gait.updatedMs = 0xfffffff0u;
  allocation::allocate(input, configuration, geometry(), {}, state, 4, 20);
  assert(std::abs(state.gait.radians - 0.02 * 2 * kPi) < 1e-6);
  configuration.tail.frequencyHz = 2;
  allocation::allocate(input, configuration, geometry(), {}, state, 24, 20);
  assert(std::abs(state.gait.radians - 0.06 * 2 * kPi) < 1e-6);
}
void pitchKeepsPriorityOverRoll() {
  auto input = request();
  input.pitchPercent = 100;
  input.rollPercent = 100;
  auto configuration = recipe();
  configuration.pectoral.leftPositivePitchDegrees = 15;
  configuration.pectoral.rightPositivePitchDegrees = 15;
  configuration.pectoral.leftRollDegrees = 10;
  configuration.pectoral.rightRollDegrees = -10;
  auto limits = geometry();
  limits.pectoralLeft = limits.pectoralRight = {90, 70, 110};
  auto state = phase(0);
  const auto result = allocation::allocate(input, configuration, limits, {}, state, 0, 20);
  assert(near(result.pectoralCorrectionScale, 0.5f));
  assert(near(result.targetFrame.pectoralLeftDegrees, 110));
  assert(near(result.targetFrame.pectoralRightDegrees, 100));
  assert(result.limitedReasons & allocation::PectoralLimited);
}
void partialCommitAndClockAreIndependent() {
  auto input = request();
  input.mode = DriveMode::Prop;
  input.surgePercent = 60;
  input.yawPercent = 20;
  input.immediate = false;
  auto state = phase(0);
  robofish::ActuatorFrame history;
  const auto first = allocation::allocate(input, recipe(), geometry(), history, state, 0, 20);
  // Simulate one failed right-prop write. The allocator never commits on its own.
  robofish::mergeWrittenFrame(history, first.candidateFrame,
      robofish::AllFrameChannels & ~robofish::PropulsionRight);
  assert(history.leftPropulsionPercent == 10 && history.rightPropulsionPercent == 0);
  const auto second = allocation::allocate(input, recipe(), geometry(), history, state, 20, 20);
  assert(second.candidateFrame.leftPropulsionPercent == 20);
  assert(second.candidateFrame.rightPropulsionPercent == 10);
  assert(state.gait.radians > 0);
  assert(history.leftPropulsionPercent == 10 && history.rightPropulsionPercent == 0);
}
void flowAssistIsExplicitAndElapsedSlewIsCapped() {
  auto input = request();
  input.pitchPercent = 50;
  auto state = phase(0);
  const auto noAssist = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(noAssist.targetFrame.leftPropulsionPercent == 0);
  input.allowFlowAssist = true;
  input.immediate = false;
  const auto assisted = allocation::allocate(input, recipe(), geometry(), {}, state, 40, 40);
  assert(assisted.targetFrame.flowAssisted);
  assert(assisted.targetFrame.leftPropulsionPercent == 50);
  assert(assisted.candidateFrame.leftPropulsionPercent == 10);
  assert(near(assisted.candidateFrame.pectoralLeftDegrees, 95));
  assert(std::abs(assisted.phaseRad - 0.04 * 1.5 * 2 * kPi) < 1e-6);
}
void independentFlowAssistOnlyRaisesAnActiveMinimum() {
  auto input = request();
  input.independentDrives = true;
  input.tailDrivePercent = 20;
  input.propulsionDrivePercent = 80;
  input.surgePercent = 80;
  input.allowFlowAssist = true;
  auto state = phase(kPi / 2);
  auto result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.tailLoad, 0.2f));
  assert(result.targetFrame.leftPropulsionPercent == 80);
  assert(!result.targetFrame.flowAssisted);
  input.pitchPercent = 50;
  result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.tailLoad, 0.5f));
  assert(result.targetFrame.leftPropulsionPercent == 80);
  assert(result.targetFrame.flowAssisted);
  input.tailEnabled = false;
  result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.tailLoad, 0));
  assert(result.targetFrame.leftPropulsionPercent == 80);
  assert(!result.targetFrame.flowAssisted);
  input.mode = DriveMode::Bionic;
  input.tailEnabled = true;
  input.tailDrivePercent = 80;
  input.propulsionDrivePercent = 20;
  result = allocation::allocate(input, recipe(), geometry(), {}, state, 0, 20);
  assert(near(result.tailLoad, 0.8f));
  assert(result.targetFrame.leftPropulsionPercent == 0);
  assert(!result.targetFrame.flowAssisted);

  input.mode = DriveMode::Hybrid;
  input.tailDrivePercent = 20;
  input.propulsionDrivePercent = 80;
  input.allowFlowAssist = false;
  auto configuration = recipe();
  configuration.diveDragCompensationPercent = 10;
  result = allocation::allocate(input, configuration, geometry(), {}, state, 0, 20);
  assert(near(result.tailLoad, 0.25f));  // 20 + 50% * 10 = 25, independently.
  assert(result.targetFrame.leftPropulsionPercent == 85);
  assert(!result.targetFrame.flowAssisted);
}
void invalidAndStoppedRequestsDoNotLeakOldTargets() {
  auto input = request();
  auto state = phase(0);
  input.surgePercent = std::numeric_limits<float>::quiet_NaN();
  const auto invalid = allocation::allocate(input, recipe(), geometry(), {}, state, 20, 20);
  assert(invalid.limitedReasons == allocation::InvalidInput);
  assert(invalid.candidateFrame.leftPropulsionPercent == 0);
  assert(std::isfinite(invalid.candidateFrame.tailFrontDegrees));
  assert(state.gait.updatedMs == 0);
  input = request();
  input.enabled = false;
  input.surgePercent = 100;
  input.pitchPercent = 100;
  input.yawPercent = 100;
  input.allowFlowAssist = true;
  const auto stopped = allocation::allocate(input, recipe(), geometry(), {}, state, 20, 20);
  assert(stopped.targetFrame.leftPropulsionPercent == 0);
  assert(near(stopped.targetFrame.tailFrontDegrees, 90));
  assert(near(stopped.targetFrame.pectoralLeftDegrees, 90));
  assert(!stopped.targetFrame.flowAssisted);

  auto badGeometry = geometry();
  badGeometry.tailFront.minimumDegrees = 160;
  const auto bad = allocation::allocate(request(), recipe(), badGeometry, {}, state, 40, 20);
  assert(bad.limitedReasons == allocation::InvalidInput);
  assert(std::isfinite(bad.candidateFrame.tailFrontDegrees));
  assert(bad.candidateFrame.leftPropulsionPercent == 0);
  assert(state.gait.updatedMs == 20);
  auto badRecipe = recipe();
  badRecipe.tail.rateDegreesPerSecond = -1;
  const auto negativeRate = allocation::allocate(request(), badRecipe, geometry(), {}, state, 40, 20);
  assert(negativeRate.limitedReasons == allocation::InvalidInput);

  auto inputWithRearDisabled = request();
  inputWithRearDisabled.surgePercent = 100;
  auto noRear = recipe();
  noRear.tail.rearAmplitudeRatio = 0;
  noRear.tail.phaseLagRad = std::numeric_limits<float>::quiet_NaN();
  const auto badPhase = allocation::allocate(inputWithRearDisabled, noRear, geometry(), {}, state, 40, 20);
  assert(badPhase.limitedReasons == allocation::InvalidInput);
  noRear.tail.phaseLagRad = 0;
  state = phase(kPi / 2);
  const auto frontOnly = allocation::allocate(inputWithRearDisabled, noRear, geometry(), {}, state, 0, 20);
  assert(near(frontOnly.targetFrame.tailFrontDegrees, 115));
  assert(near(frontOnly.targetFrame.tailRearDegrees, 90));

  auto overflowRecipe = recipe();
  overflowRecipe.pectoral.leftRollDegrees = std::numeric_limits<float>::max();
  overflowRecipe.pectoral.leftYawDegrees = std::numeric_limits<float>::max();
  auto overflowInput = request();
  overflowInput.rollPercent = overflowInput.yawPercent = 100;
  const auto overflow = allocation::allocate(overflowInput, overflowRecipe, geometry(), {}, state, 20, 20);
  assert(overflow.limitedReasons & allocation::InvalidInput);
  assert(near(overflow.candidateFrame.pectoralLeftDegrees, 90));
}
void integerPropulsionSlewNeverExceedsItsBudget() {
  auto input = request();
  input.mode = DriveMode::Prop;
  input.immediate = false;
  auto configuration = recipe();
  configuration.propulsion.ratePercentPerSecond = 25;  // 0.5% per 20 ms.
  robofish::ActuatorFrame history;
  history.leftPropulsionPercent = 5;
  history.rightPropulsionPercent = -5;
  auto state = phase(0);
  const auto held = allocation::allocate(input, configuration, geometry(), history, state, 20, 20);
  assert(held.candidateFrame.leftPropulsionPercent == 5);
  assert(held.candidateFrame.rightPropulsionPercent == -5);
  assert(held.limitedReasons & allocation::PropulsionRateLimited);
  configuration.propulsion.ratePercentPerSecond = 50;  // Exactly one nominal step.
  const auto moved = allocation::allocate(input, configuration, geometry(), history, state, 40, 20);
  assert(moved.candidateFrame.leftPropulsionPercent == 4);
  assert(moved.candidateFrame.rightPropulsionPercent == -4);
}
}  // namespace

int main() {
  sixChannelCompositionAndSlew();
  independentContributionsAndCommonPriority();
  reverseIsNotPhaseInversion();
  unusedDrivesCannotChangeTheExecutedDirection();
  tailGeometryAndPhaseAreShared();
  pitchKeepsPriorityOverRoll();
  partialCommitAndClockAreIndependent();
  flowAssistIsExplicitAndElapsedSlewIsCapped();
  independentFlowAssistOnlyRaisesAnActiveMinimum();
  invalidAndStoppedRequestsDoNotLeakOldTargets();
  integerPropulsionSlewNeverExceedsItsBudget();
  std::cout << "PASS: L2 allocation tree, reverse semantics, constraints and partial commits\n";
}
