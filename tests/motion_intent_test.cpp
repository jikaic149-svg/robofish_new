#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "config/MotionAllocationConfig.h"
#include "control/l0/IntentAdapter.h"
#include "control/l0/VisionMotionPolicy.h"

namespace intent = robofish::control::l0;
namespace {
bool near(float left, float right) { return std::abs(left - right) < 0.0001f; }
void remoteDirectionsAndGearRemainExplicit() {
  intent::BodyMotionTarget remote;
  remote.mode = DriveMode::Hybrid;
  remote.enabled = true;
  remote.surgePercent = 80;
  remote.yawPercent = 30;
  remote.heavePercent = -50;
  remote.knob = 500;
  remote.allowFlowAssist = true;
  auto adapted = intent::fromRemote(remote, false);
  assert(adapted.mode == DriveMode::Hybrid && adapted.body.enabled);
  assert(near(adapted.body.surgePercent, 80));
  assert(near(adapted.body.yawPercent, 30));
  assert(near(adapted.body.pitchPercent, -50));
  assert(near(adapted.outputScale, 0.75f));
  assert(adapted.allowFlowAssist && !adapted.independentDrives);
  assert(adapted.independentTailYaw && near(adapted.tailYawPercent, 30));
  remote.propulsionReversed = remote.steeringReversed = remote.tailReversed = true;
  adapted = intent::fromRemote(remote, true);
  assert(near(adapted.body.surgePercent, -80));
  assert(near(adapted.body.yawPercent, -30));
  assert(near(adapted.tailYawPercent, 30));
  assert(adapted.immediate);
  remote.knob = 65535;
  remote.enabled = false;
  adapted = intent::fromRemote(remote, false);
  assert(near(adapted.outputScale, 1));
  assert(!adapted.body.enabled && !adapted.allowFlowAssist);
  remote.surgePercent = std::numeric_limits<float>::quiet_NaN();
  adapted = intent::fromRemote(remote, false);
  assert(robofish::control::l1::validateCommand(adapted.body) ==
      robofish::control::l1::CommandValidation::NonFinite);
}
void manualDriveDemandsAndSteeringStayIndependent() {
  intent::ManualIntent manual{DriveMode::Hybrid, MotionAction::Left,
      74, 26, 80, true, true, false, true, false, true, false, 123};
  auto adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(adapted.independentDrives && adapted.independentTailYaw);
  assert(near(adapted.tailDrivePercent, 74));
  assert(near(adapted.propulsionDrivePercent, 26));
  assert(near(adapted.body.surgePercent, 26));
  assert(near(adapted.body.yawPercent, 80));
  assert(near(adapted.tailYawPercent, -80));
  assert(!adapted.allowFlowAssist && near(adapted.body.pitchPercent, 0));
  manual.reverse = true;
  adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(near(adapted.tailDrivePercent, -74));
  assert(near(adapted.propulsionDrivePercent, -26));
  manual.escReversed = true;
  adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(near(adapted.propulsionDrivePercent, 26));
  assert(near(adapted.tailDrivePercent, -74));
  manual.tailMoving = manual.propulsionMoving = false;
  adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(!adapted.tailEnabled && !adapted.propulsionEnabled);
  assert(adapted.tailDrivePercent == 0 && adapted.propulsionDrivePercent == 0);

  manual = {DriveMode::Bionic, MotionAction::Forward, 80, 80, 0,
      true, true, false, false, true, false, false, 0};
  adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(adapted.tailEnabled && !adapted.propulsionEnabled);
  assert(adapted.tailDrivePercent == 80 && adapted.propulsionDrivePercent == 0);
  assert(adapted.body.surgePercent == 80);
  manual.mode = DriveMode::Prop;
  manual.reverse = true;
  adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(adapted.tailDrivePercent == 0);  // Static rudder has no propulsion drive.
  assert(adapted.propulsionDrivePercent == 80);  // reverse XOR preference.
}
void pitchCompatibilityAndSurfaceOutput() {
  intent::ManualIntent manual;
  auto adapted = intent::fromManual(manual, RF_PITCH_CENTER, false);
  assert(adapted.body.enabled && adapted.mode == DriveMode::Prop);
  assert(adapted.body.pitchPercent == 0);
  assert(!adapted.tailEnabled && !adapted.propulsionEnabled);
  assert(intent::fromManual(manual, RF_PITCH_UI_NEUTRAL, false).body.pitchPercent == 0);
  assert(near(intent::fromManual(manual, RF_PITCH_MIN_ANGLE, false).body.pitchPercent, 100));
  assert(near(intent::fromManual(manual, RF_PITCH_MAX_ANGLE, false).body.pitchPercent, -100));
  assert(near(intent::fromManual(manual, -10000, false).body.pitchPercent, 100));
  manual = {DriveMode::Hybrid, MotionAction::Forward, 100, 100, 100,
      true, true, false, true, false, false, false, 0};
  adapted = intent::fromManual(manual, RF_PITCH_CENTER, true);
  assert(adapted.mode == DriveMode::Prop && adapted.body.pitchPercent == 100);
  assert(adapted.body.surgePercent == 0 && adapted.body.yawPercent == 0);
  assert(adapted.tailDrivePercent == 0 && adapted.propulsionDrivePercent == 0);
  assert(!adapted.tailEnabled && !adapted.propulsionEnabled);
  assert(!adapted.allowFlowAssist && adapted.immediate);
}
void oneHardwareProfileForEverySource() {
  const auto geometry = robofish::config::motion::makeGeometry();
  assert(geometry.tailFront.centerDegrees == 90);
  assert(geometry.tailRear.centerDegrees == 90);
  const auto bionic = robofish::config::motion::makeRecipe(DriveMode::Bionic);
  const auto hybrid = robofish::config::motion::makeRecipe(DriveMode::Hybrid);
  assert(near(bionic.tail.amplitudeDegrees, 25));
  assert(near(hybrid.tail.amplitudeDegrees, 25));
  assert(near(bionic.tail.frequencyHz, 1.5f));
  assert(near(hybrid.tail.frequencyHz, 1.5f));
  assert(near(bionic.tail.rearAmplitudeRatio, 0.9f));
  assert(near(bionic.tail.phaseLagRad, static_cast<float>(robofish::algorithm::kTwoPi / 6)));
  assert(near(hybrid.tail.phaseLagRad, static_cast<float>(robofish::algorithm::kTwoPi / 5)));
  assert(near(hybrid.tail.rateDegreesPerSecond, 250));
  assert(near(hybrid.propulsion.ratePercentPerSecond, 500));
  assert(near(hybrid.propulsion.yawGain, 1));
  assert(near(hybrid.pectoral.leftPositivePitchDegrees, 42));
  assert(near(hybrid.pectoral.rightPositivePitchDegrees, -42));
  assert(near(hybrid.pectoral.leftNegativePitchDegrees, -34));
  assert(near(hybrid.pectoral.rightNegativePitchDegrees, 34));
  assert(hybrid.pectoral.leftRollDegrees == 0 && hybrid.pectoral.rightRollDegrees == 0);
  assert(hybrid.maximumSlewElapsedMs == 20);
}
void visionPolicyRemainsAnIntentOnlyMapping() {
  const auto straight = intent::mapVisionDemand(0, 100, 0);
  assert(straight.action == MotionAction::Forward);
  assert(straight.tailPowerPercent == RF_VISION_TAIL_STRAIGHT_POWER_PERCENT);
  assert(straight.propulsionPowerPercent == 100);
  const auto curve = intent::mapVisionDemand(100, 100, 1);
  assert(curve.action == MotionAction::Left);  // Existing PID sign convention.
  assert(curve.tailPowerPercent == RF_VISION_TAIL_TURN_POWER_PERCENT);
  assert(curve.propulsionPowerPercent == RF_VISION_PROP_CURVE_MIN_POWER_PERCENT);
  assert(intent::mapVisionDemand(-100, 50, 1).action == MotionAction::Right);
  const auto stop = intent::mapVisionDemand(100, 0, 1);
  assert(stop.action == MotionAction::Stopped && stop.tailPowerPercent == 0);
  assert(intent::mapVisionDemand(std::numeric_limits<float>::quiet_NaN(), 100, 0).action ==
      MotionAction::Stopped);
  assert(intent::mapVisionDemand(0, 100, 10).propulsionPowerPercent ==
      RF_VISION_PROP_CURVE_MIN_POWER_PERCENT);
}
}  // namespace

int main() {
  remoteDirectionsAndGearRemainExplicit();
  manualDriveDemandsAndSteeringStayIndependent();
  pitchCompatibilityAndSurfaceOutput();
  oneHardwareProfileForEverySource();
  visionPolicyRemainsAnIntentOnlyMapping();
  std::cout << "PASS: L0 intent adapters, source-independent profile and vision intent policy\n";
}
