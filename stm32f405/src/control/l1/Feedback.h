#pragma once

#include <cmath>
#include "channel/AttitudeSample.h"
#include "channel/ControlFeedback.h"

namespace robofish::control::l1 {

enum class SampleValidation : uint8_t {
  Valid, NonFinite, FutureTimestamp, StaleSequence, ReorderedTimestamp
};

namespace detail {
constexpr uint32_t kHalfRange = uint32_t(1) << 31;

inline bool finiteSample(const AttitudeSample &sample) {
  return std::isfinite(sample.rollRad) && std::isfinite(sample.pitchRad) &&
      std::isfinite(sample.yawRad) && std::isfinite(sample.rollRateRadPerSec) &&
      std::isfinite(sample.pitchRateRadPerSec) && std::isfinite(sample.yawRateRadPerSec);
}
inline bool finiteSample(const DepthSample &sample) {
  return std::isfinite(sample.depthMeters);
}
inline bool finiteSample(const VelocitySample &sample) {
  return std::isfinite(sample.surgeMetersPerSecond);
}
}  // namespace detail

// Null previous accepts the first sequence, including zero. Comparisons use
// unsigned half-range ordering: observations/sequence gaps must be <2^31.
// A producer restart needs the owner's explicit reset of that stream history.
template <typename Sample>
inline SampleValidation validateSample(const Sample &sample,
    const Sample *previous, uint32_t nowMs) {
  if (!detail::finiteSample(sample)) return SampleValidation::NonFinite;
  if (nowMs - sample.sampledAtMs >= detail::kHalfRange)
    return SampleValidation::FutureTimestamp;
  if (previous) {
    const uint32_t advance = sample.sequence - previous->sequence;
    if (!advance || advance >= detail::kHalfRange)
      return SampleValidation::StaleSequence;
    if (sample.sampledAtMs - previous->sampledAtMs >= detail::kHalfRange)
      return SampleValidation::ReorderedTimestamp;
  }
  return SampleValidation::Valid;
}

// Explicit FLU -> FRD basis change on BOTH reference and body frames for the
// existing paired-frame ZYX producer contract (not arbitrary ENU attitude):
// roll stays the same; pitch/yaw and q/r body rates change sign.
inline AttitudeFeedback attitudeToFrd(const AttitudeSample &sample) {
  return {sample.sequence, sample.sampledAtMs, sample.rollRad,
      -sample.pitchRad, -sample.yawRad, sample.rollRateRadPerSec,
      -sample.pitchRateRadPerSec, -sample.yawRateRadPerSec};
}

template <typename Sample>
inline bool sampleFresh(const Sample &sample, uint32_t nowMs, uint32_t maximumAgeMs) {
  return validateSample(sample, static_cast<const Sample *>(nullptr), nowMs) ==
      SampleValidation::Valid && nowMs - sample.sampledAtMs <= maximumAgeMs;
}

// Null pointers mean no observation. Old values remain inspectable but are
// marked unavailable. Copying/aging does not mutate or renew any input sample.
inline ControlFeedback feedbackSnapshot(const AttitudeSample *attitude,
    const DepthSample *depth, const VelocitySample *velocity,
    uint32_t nowMs, uint32_t maximumAgeMs) {
  ControlFeedback snapshot;
  snapshot.snapshotAtMs = nowMs;
  if (attitude) {
    snapshot.attitude = attitudeToFrd(*attitude);
    snapshot.attitudeAgeMs = nowMs - attitude->sampledAtMs;
    snapshot.attitudeAvailable = sampleFresh(*attitude, nowMs, maximumAgeMs);
  }
  if (depth) {
    snapshot.depth = *depth;
    snapshot.depthAgeMs = nowMs - depth->sampledAtMs;
    snapshot.depthAvailable = sampleFresh(*depth, nowMs, maximumAgeMs);
  }
  if (velocity) {
    snapshot.velocity = *velocity;
    snapshot.velocityAgeMs = nowMs - velocity->sampledAtMs;
    snapshot.velocityAvailable = sampleFresh(*velocity, nowMs, maximumAgeMs);
  }
  return snapshot;
}

}  // namespace robofish::control::l1
