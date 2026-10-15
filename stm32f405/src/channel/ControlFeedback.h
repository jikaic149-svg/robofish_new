#pragma once

#include <stdint.h>

namespace robofish {

// All timestamps are acquisition times translated at the input boundary into
// the F405 local monotonic millisecond clock. Remote boot clocks are not valid.
// Each stream owns its sequence; wrap is allowed, reboot/reset is explicit.
struct DepthSample {
  uint32_t sequence = 0;
  uint32_t sampledAtMs = 0;
  float depthMeters = 0;  // Down-positive from the producer's declared datum.
};
struct VelocitySample {
  uint32_t sequence = 0;
  uint32_t sampledAtMs = 0;
  // Forward-positive body-axis velocity. Producer and consumer must agree on
  // water-relative versus ground-relative measurement; RPM/current is neither.
  float surgeMetersPerSecond = 0;
};

// Paired FRD (forward/right/down) reference and body frames; right-handed ZYX
// Euler angles in radians. An arbitrary ENU-world attitude needs conversion.
// Positive roll: right side down; pitch: nose up; yaw: right turn.
// Angular rates are body-axis rates in rad/s, not Euler-angle derivatives.
struct AttitudeFeedback {
  uint32_t sequence = 0;
  uint32_t sampledAtMs = 0;
  float rollRad = 0;
  float pitchRad = 0;
  float yawRad = 0;
  float rollRateRadPerSec = 0;
  float pitchRateRadPerSec = 0;
  float yawRateRadPerSec = 0;
};

// A copied observation snapshot. The owner creates it under its existing
// lock; algorithms consume const snapshots. Values are usable only when the
// corresponding Available flag is true. No observation is fabricated as valid.
struct ControlFeedback {
  uint32_t snapshotAtMs = 0;
  AttitudeFeedback attitude;
  DepthSample depth;
  VelocitySample velocity;
  bool attitudeAvailable = false;
  bool depthAvailable = false;
  bool velocityAvailable = false;
  uint32_t attitudeAgeMs = UINT32_MAX;
  uint32_t depthAgeMs = UINT32_MAX;
  uint32_t velocityAgeMs = UINT32_MAX;
};

}  // namespace robofish
