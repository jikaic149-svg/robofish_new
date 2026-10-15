#pragma once

#include <cmath>
#include <stdint.h>

namespace robofish::control::l1 {

// Normalized body requests, not force, angular rate or attitude setpoints.
// +surge forward, +yaw right, +pitch nose up, +roll right side down.
// This layer knows no source, mode, hardware or scheduler.
struct BodyCommand {
  float surgePercent = 0;
  float yawPercent = 0;
  float pitchPercent = 0;
  float rollPercent = 0;
  bool enabled = false;
};

enum class CommandValidation : uint8_t { Valid, NonFinite, OutOfRange };

// New callers must provide [-100,100]. No implicit clipping: compatibility
// adapters own any deliberate clipping of an older input representation.
inline CommandValidation validateCommand(const BodyCommand &command) {
  const float axes[] = {command.surgePercent, command.yawPercent,
      command.pitchPercent, command.rollPercent};
  for (float value : axes)
    if (!std::isfinite(value)) return CommandValidation::NonFinite;
  for (float value : axes)
    if (value < -100 || value > 100) return CommandValidation::OutOfRange;
  return CommandValidation::Valid;
}

}  // namespace robofish::control::l1
