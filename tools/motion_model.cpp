#include <cmath>
#include "config/MotionAllocationConfig.h"
#include "control/l0/IntentAdapter.h"
#include "control/l1/Command.h"
#include "control/l2/Allocator.h"
#include "device/motion/EscCalibration.h"

extern "C" __declspec(dllexport) int rf_pc_allocate(int mode, float surge,
    float yaw, float heave, unsigned now, float *output) {
  if (!output || mode < 1 || mode > 3 || !std::isfinite(surge) ||
      !std::isfinite(yaw) || !std::isfinite(heave)) return 0;
  robofish::control::l0::BodyMotionTarget target;
  target.mode = static_cast<decltype(target.mode)>(mode);
  target.enabled = true;
  target.knob = 1000;
  target.surgePercent = surge;
  target.yawPercent = yaw;
  target.heavePercent = heave;
  target.allowFlowAssist = false;
  const auto intent = robofish::control::l0::fromRemote(target, true);
  if (robofish::control::l1::validateCommand(intent.body) !=
      robofish::control::l1::CommandValidation::Valid) return 0;
  robofish::control::l2::Request request;
  request.mode = intent.mode;
  request.surgePercent = intent.body.surgePercent;
  request.yawPercent = intent.body.yawPercent;
  request.pitchPercent = intent.body.pitchPercent;
  request.rollPercent = intent.body.rollPercent;
  request.enabled = intent.body.enabled;
  request.outputScale = intent.outputScale;
  request.allowFlowAssist = intent.allowFlowAssist;
  request.independentTailYaw = intent.independentTailYaw;
  request.tailYawPercent = intent.tailYawPercent;
  request.immediate = true;
  // The historical ABI has no session handle. Keep this a stateless target
  // preview: L2 owns phase generation from its common time origin, while each
  // call owns local phase/nominal state. Never share state between workstations.
  // Actual task slew, leases and write failures are tested in the firmware
  // pipeline, not simulated by this six-output preview.
  robofish::control::l2::State state;
  const robofish::ActuatorFrame nominal;
  const auto allocation = robofish::control::l2::allocate(request,
      robofish::config::motion::makeRecipe(request.mode),
      robofish::config::motion::makeGeometry(), nominal, state, now,
      robofish::config::control::kPeriodMs);
  const auto &frame = allocation.targetFrame;
  output[0] = frame.tailFrontDegrees;
  output[1] = frame.tailRearDegrees;
  output[2] = frame.pectoralLeftDegrees;
  output[3] = frame.pectoralRightDegrees;
  // This ABI carries no measured battery voltage. Preview at the nominal
  // reference voltage, using the same installed L3 mapping as the real writer.
  output[4] = robofish::device::propulsionElectricalPercent(
      frame.leftPropulsionPercent, robofish::device::PropulsionSide::Left,
      robofish::config::propulsion::kReferenceVoltageMv);
  output[5] = robofish::device::propulsionElectricalPercent(
      frame.rightPropulsionPercent, robofish::device::PropulsionSide::Right,
      robofish::config::propulsion::kReferenceVoltageMv);
  return 1;
}
