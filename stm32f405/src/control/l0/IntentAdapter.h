#pragma once

#include <stdint.h>

#include "channel/MotionTypes.h"
#include "control/l1/Command.h"

namespace robofish::control::l0 {

// Compatibility representation for the existing remote command owner.
// heavePercent historically means nose-up/down fin request, not direct heave.
struct BodyMotionTarget {
  DriveMode mode = DriveMode::Idle;
  float surgePercent = 0;
  float yawPercent = 0;
  float heavePercent = 0;
  uint16_t knob = 0;
  bool enabled = false;
  bool allowFlowAssist = false;
  bool tailReversed = false;
  bool propulsionReversed = false;
  bool steeringReversed = false;
};

// Aggregate field order preserves existing task construction. Independent drive
// powers remain intentional upstream choices; no actuator angle is carried here.
struct ManualIntent {
  DriveMode mode = DriveMode::Idle;
  MotionAction action = MotionAction::Stopped;
  uint8_t tailPowerPercent = 0;
  uint8_t propulsionPowerPercent = 0;
  uint8_t turnStrengthPercent = 0;
  bool tailMoving = false;
  bool propulsionMoving = false;
  bool reverse = false;
  bool turning = false;
  bool escReversed = false;
  bool steeringReversed = false;
  bool tailReversed = false;
  uint32_t nowMs = 0;  // Retained aggregate compatibility; no private phase clock.
};

struct AdaptedIntent {
  l1::BodyCommand body;
  DriveMode mode = DriveMode::Idle;
  float outputScale = 1;
  bool allowFlowAssist = false;
  bool immediate = false;
  bool independentDrives = false;
  float tailDrivePercent = 0;
  float propulsionDrivePercent = 0;
  bool tailEnabled = true;
  bool propulsionEnabled = true;
  bool independentTailYaw = true;
  float tailYawPercent = 0;
};

AdaptedIntent fromRemote(const BodyMotionTarget &target, bool immediate);
AdaptedIntent fromManual(const ManualIntent &intent, int pitch, bool failsafe);

}  // namespace robofish::control::l0
