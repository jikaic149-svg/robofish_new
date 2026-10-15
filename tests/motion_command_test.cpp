#include <cassert>
#include <array>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

#include "control/l1/Command.h"
#include "control/l1/Feedback.h"

namespace {
// Independent ZYX rotation matrix to check the physical basis change, rather
// than only restating the converter's three sign assignments.
std::array<float, 9> rotation(float roll, float pitch, float yaw) {
  const float sr = std::sin(roll), cr = std::cos(roll);
  const float sp = std::sin(pitch), cp = std::cos(pitch);
  const float sy = std::sin(yaw), cy = std::cos(yaw);
  return {cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
      sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
      -sp, cp*sr, cp*cr};
}
}

int main() {
  using namespace robofish;
  using namespace robofish::control::l1;
  BodyCommand command;
  assert(validateCommand(command) == CommandValidation::Valid);
  command = {100, -100, 100, -100, true};
  assert(validateCommand(command) == CommandValidation::Valid);
  for (unsigned axis = 0; axis < 4; ++axis) {
    float *axes[] = {&command.surgePercent, &command.yawPercent,
        &command.pitchPercent, &command.rollPercent};
    const float previous = *axes[axis];
    for (float value : {100.1f, -100.1f}) {
      *axes[axis] = value;
      assert(validateCommand(command) == CommandValidation::OutOfRange);
      assert(*axes[axis] == value); // Reject, do not silently clamp the input.
    }
    *axes[axis] = std::numeric_limits<float>::quiet_NaN();
    assert(validateCommand(command) == CommandValidation::NonFinite);
    *axes[axis] = std::numeric_limits<float>::infinity();
    command.enabled = false;
    assert(validateCommand(command) == CommandValidation::NonFinite);
    command.enabled = true;
    *axes[axis] = previous;
  }

  const AttitudeSample *noAttitude = nullptr;
  const DepthSample *noDepth = nullptr;
  const VelocitySample *noVelocity = nullptr;
  auto snapshot = feedbackSnapshot(noAttitude, noDepth, noVelocity, 100, 50);
  assert(snapshot.snapshotAtMs == 100);
  assert(!snapshot.attitudeAvailable && !snapshot.depthAvailable && !snapshot.velocityAvailable);
  assert(snapshot.attitudeAgeMs == UINT32_MAX && snapshot.depthAgeMs == UINT32_MAX);

  AttitudeSample attitude{1, 100, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
  DepthSample depth{1, 100, 1.25f};
  VelocitySample velocity{1, 100, -0.4f};
  assert(validateSample(attitude, noAttitude, 100) == SampleValidation::Valid);
  assert(validateSample(depth, noDepth, 100) == SampleValidation::Valid);
  assert(validateSample(velocity, noVelocity, 100) == SampleValidation::Valid);
  snapshot = feedbackSnapshot(&attitude, &depth, &velocity, 150, 50);
  assert(snapshot.attitudeAvailable && snapshot.depthAvailable && snapshot.velocityAvailable);
  assert(snapshot.attitudeAgeMs == 50 && snapshot.depthAgeMs == 50 && snapshot.velocityAgeMs == 50);
  assert(snapshot.attitude.sequence == 1 && snapshot.attitude.sampledAtMs == 100);
  assert(snapshot.attitude.rollRad == attitude.rollRad);
  assert(snapshot.attitude.pitchRad == -attitude.pitchRad);
  assert(snapshot.attitude.yawRad == -attitude.yawRad);
  assert(snapshot.attitude.rollRateRadPerSec == attitude.rollRateRadPerSec);
  assert(snapshot.attitude.pitchRateRadPerSec == -attitude.pitchRateRadPerSec);
  assert(snapshot.attitude.yawRateRadPerSec == -attitude.yawRateRadPerSec);
  const auto flu = rotation(attitude.rollRad, attitude.pitchRad, attitude.yawRad);
  const auto frd = rotation(snapshot.attitude.rollRad,
      snapshot.attitude.pitchRad, snapshot.attitude.yawRad);
  const float basis[] = {1, -1, -1}; // FRD = diag(1,-1,-1) * FLU
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned column = 0; column < 3; ++column)
      assert(std::abs(frd[row*3+column] - basis[row]*flu[row*3+column]*basis[column]) < 1e-6f);
  assert(snapshot.depth.depthMeters == 1.25f);
  assert(snapshot.velocity.surgeMetersPerSecond == -0.4f);
  snapshot = feedbackSnapshot(&attitude, &depth, &velocity, 151, 50);
  assert(!snapshot.attitudeAvailable && !snapshot.depthAvailable && !snapshot.velocityAvailable);
  assert(snapshot.depth.depthMeters == 1.25f); // Stale value is inspectable, never valid.
  assert(attitude.pitchRad == 0.2f && depth.depthMeters == 1.25f); // Inputs unchanged.

  auto newerDepth = depth;
  assert(validateSample(newerDepth, &depth, 150) == SampleValidation::StaleSequence);
  newerDepth.sequence = 0;
  assert(validateSample(newerDepth, &depth, 150) == SampleValidation::StaleSequence);
  newerDepth.sequence = 2;
  assert(validateSample(newerDepth, &depth, 150) == SampleValidation::Valid); // Same millisecond allowed.
  newerDepth.sampledAtMs = 99;
  assert(validateSample(newerDepth, &depth, 150) == SampleValidation::ReorderedTimestamp);
  newerDepth.sampledAtMs = 151;
  assert(validateSample(newerDepth, &depth, 150) == SampleValidation::FutureTimestamp);
  newerDepth.sequence = depth.sequence + (uint32_t(1) << 31);
  newerDepth.sampledAtMs = depth.sampledAtMs;
  assert(validateSample(newerDepth, &depth, 150) == SampleValidation::StaleSequence);

  // Sequence and clock wrap independently, without signed casts or wall clocks.
  DepthSample previous{UINT32_MAX, 0xfffffff0U, 0.5f};
  DepthSample wrapped{0, 0x10U, 0.6f};
  assert(validateSample(wrapped, &previous, 0x20U) == SampleValidation::Valid);
  snapshot = feedbackSnapshot(noAttitude, &previous, noVelocity, 0x10U, 32);
  assert(snapshot.depthAvailable && snapshot.depthAgeMs == 32);
  snapshot = feedbackSnapshot(noAttitude, &previous, noVelocity, 0x11U, 32);
  assert(!snapshot.depthAvailable && snapshot.depthAgeMs == 33);
  auto future = wrapped;
  future.sampledAtMs = 0x21U;
  assert(validateSample(future, noDepth, 0x20U) == SampleValidation::FutureTimestamp);
  snapshot = feedbackSnapshot(noAttitude, &future, noVelocity, 0x20U, UINT32_MAX);
  assert(!snapshot.depthAvailable); // A large age policy cannot make future data valid.
  auto ambiguous = wrapped;
  ambiguous.sampledAtMs = uint32_t(1) << 31;
  assert(validateSample(ambiguous, noDepth, 0) == SampleValidation::FutureTimestamp);
  DepthSample zero{};
  assert(validateSample(zero, noDepth, 0) == SampleValidation::Valid);

  for (unsigned component = 0; component < 6; ++component) {
    auto invalid = attitude;
    float *components[] = {&invalid.rollRad, &invalid.pitchRad, &invalid.yawRad,
        &invalid.rollRateRadPerSec, &invalid.pitchRateRadPerSec, &invalid.yawRateRadPerSec};
    *components[component] = std::numeric_limits<float>::quiet_NaN();
    assert(validateSample(invalid, noAttitude, 100) == SampleValidation::NonFinite);
    snapshot = feedbackSnapshot(&invalid, noDepth, noVelocity, 100, 100);
    assert(!snapshot.attitudeAvailable);
  }
  depth.depthMeters = std::numeric_limits<float>::infinity();
  velocity.surgeMetersPerSecond = std::numeric_limits<float>::quiet_NaN();
  assert(validateSample(depth, noDepth, 100) == SampleValidation::NonFinite);
  assert(validateSample(velocity, noVelocity, 100) == SampleValidation::NonFinite);
  snapshot = feedbackSnapshot(noAttitude, &depth, &velocity, 100, 100);
  assert(!snapshot.depthAvailable && !snapshot.velocityAvailable);

  std::puts("PASS: L1 four-axis validation, local sample ordering/wrap, FLU-to-FRD and read-only freshness snapshots");
}
