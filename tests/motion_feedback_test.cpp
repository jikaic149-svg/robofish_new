#include <atomic>
#include <cassert>
#include <cstdio>
#include <limits>

#define private public
#include "task/MotionController.h"
#undef private

int main() {
  using namespace robofish;
  RoboFishDriver driver;
  MotionController control(driver);
  ControlFeedback feedback;
  feedback.snapshotAtMs = 42;
  AttitudeSample attitude{7, 100, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
  DepthSample depth{11, 100, 1.5f};
  VelocitySample velocity{2, 100, -0.25f};
  assert(!control.getFeedback(feedback, 100) && feedback.snapshotAtMs == 42);
  assert(!control.submitAttitude(attitude) && !control.submitDepth(depth));
  assert(!control.submitVelocity(velocity) && !control.clearFeedback());
  assert(control.begin());
  assert(control.getFeedback(feedback, 100));
  assert(!feedback.attitudeAvailable && !feedback.depthAvailable && !feedback.velocityAvailable);

  mockTick = 100;
  assert(control.submitAttitude(attitude));
  assert(control.submitDepth(depth));
  assert(control.submitVelocity(velocity));
  assert(control.getFeedback(feedback, 50));
  assert(feedback.attitudeAvailable && feedback.depthAvailable && feedback.velocityAvailable);
  assert(feedback.attitude.rollRad == 0.1f && feedback.attitude.pitchRad == -0.2f);
  assert(feedback.attitude.yawRad == -0.3f && feedback.attitude.yawRateRadPerSec == -0.6f);
  assert(feedback.depth.depthMeters == 1.5f && feedback.velocity.surgeMetersPerSecond == -0.25f);
  assert(feedback.depth.sequence == 11 && feedback.velocity.sequence == 2);

  assert(!control.submitAttitude(attitude));
  assert(!control.submitDepth(depth));
  assert(!control.submitVelocity(velocity));
  auto futureDepth = depth;
  futureDepth.sequence = 12;
  futureDepth.sampledAtMs = 101;
  assert(!control.submitDepth(futureDepth));
  auto invalidVelocity = velocity;
  invalidVelocity.sequence = 3;
  invalidVelocity.surgeMetersPerSecond = std::numeric_limits<float>::quiet_NaN();
  assert(!control.submitVelocity(invalidVelocity));
  auto invalidAttitude = attitude;
  invalidAttitude.sequence = 8;
  invalidAttitude.pitchRad = std::numeric_limits<float>::infinity();
  assert(!control.submitAttitude(invalidAttitude));
  assert(control.getFeedback(feedback, 50));
  assert(feedback.attitude.sequence == 7 && feedback.depth.sequence == 11 && feedback.velocity.sequence == 2);

  mockTick = 150;
  assert(control.getFeedback(feedback, 50) && feedback.attitudeAvailable && feedback.depthAvailable);
  mockTick = 151;
  assert(control.getFeedback(feedback, 50));
  assert(!feedback.attitudeAvailable && !feedback.depthAvailable && !feedback.velocityAvailable);
  assert(feedback.depthAgeMs == 51 && feedback.depth.depthMeters == 1.5f);

  assert(control.clearAttitude());
  assert(control.getFeedback(feedback, 100));
  assert(!feedback.attitudeAvailable && feedback.depthAvailable && feedback.velocityAvailable);
  attitude.sequence = 0;
  attitude.sampledAtMs = 151;
  assert(control.submitAttitude(attitude));
  assert(control.clearFeedback());
  assert(control.getFeedback(feedback, 100));
  assert(!feedback.attitudeAvailable && !feedback.depthAvailable && !feedback.velocityAvailable);
  depth = {0, 151, 0.5f};
  velocity = {0, 151, 0.4f};
  assert(control.submitDepth(depth) && control.submitVelocity(velocity));
  assert(control.submitAttitude(attitude));
  assert(control.state_.safety.mode == SafetyMode::WaitCommand);
  assert(driver.attempts == 0); // Sensor intake never authorizes or writes motion.

  control.stateMutex_->locked = true;
  feedback.snapshotAtMs = 987;
  depth.sequence = 1;
  assert(!control.submitDepth(depth));
  assert(!control.getFeedback(feedback, 100) && feedback.snapshotAtMs == 987);
  assert(!control.clearFeedback());
  control.stateMutex_->locked = false;
  assert(control.getFeedback(feedback, 100) && feedback.depth.sequence == 0);

  assert(control.clearFeedback());
  mockTick = 0xfffffff0U;
  depth = {UINT32_MAX, mockTick, 0.8f};
  assert(control.submitDepth(depth));
  mockTick = 0x10U;
  assert(control.getFeedback(feedback, 32) && feedback.depthAvailable && feedback.depthAgeMs == 32);
  depth = {0, mockTick, 0.9f};
  assert(control.submitDepth(depth));
  assert(control.getFeedback(feedback, 0) && feedback.depthAvailable && feedback.depth.depthMeters == 0.9f);

  std::puts("PASS: Task sensor intake, independent histories, FRD snapshot, age, lock failure and explicit producer reset");
}
