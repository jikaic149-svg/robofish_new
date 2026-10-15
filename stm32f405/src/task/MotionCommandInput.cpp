#include "MotionController.h"

#include "control/l1/Command.h"

bool MotionController::submitBodyCommand(DriveMode mode,
    const robofish::control::l1::BodyCommand &command, uint32_t generation) {
  if (!commandAllowed() ||
      robofish::control::l1::validateCommand(command) !=
          robofish::control::l1::CommandValidation::Valid ||
      (mode != DriveMode::Bionic && mode != DriveMode::Prop && mode != DriveMode::Hybrid) ||
      !takeState(kCommandLockTimeout)) return false;
  if (!automationAllowedLocked(generation)) {
    giveState();
    return false;
  }
  if (!command.enabled) {
    exitControlLocked();
  } else {
    // Replace the previous goal rather than leaving a dormant manual/vision
    // request to resume when a later auxiliary command exits body control.
    clearTargetsLocked();
    state_.bodyTarget = command;
    state_.bodyActive = true;
    state_.drive.mode = mode;
  }
  giveState();
  return true;
}
