#include "MotionController.h"

#include "control/l1/Feedback.h"

bool MotionController::submitAttitude(const AttitudeSample &sample) {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  auto &history = state_.diagnostics;
  const auto result = robofish::control::l1::validateSample(sample,
      history.attitudeAvailable ? &history.attitude : nullptr, nowMs());
  if (result == robofish::control::l1::SampleValidation::Valid) {
    history.attitude = sample;
    history.attitudeAvailable = true;
  }
  giveState();
  return result == robofish::control::l1::SampleValidation::Valid;
}

bool MotionController::clearAttitude() {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  state_.diagnostics.attitudeAvailable = false;
  giveState();
  return true;
}

bool MotionController::submitDepth(const robofish::DepthSample &sample) {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const auto result = robofish::control::l1::validateSample(sample,
      state_.depthSeen ? &state_.depthSample : nullptr, nowMs());
  if (result == robofish::control::l1::SampleValidation::Valid) {
    state_.depthSample = sample;
    state_.depthSeen = true;
  }
  giveState();
  return result == robofish::control::l1::SampleValidation::Valid;
}

bool MotionController::submitVelocity(const robofish::VelocitySample &sample) {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const auto result = robofish::control::l1::validateSample(sample,
      state_.velocitySeen ? &state_.velocitySample : nullptr, nowMs());
  if (result == robofish::control::l1::SampleValidation::Valid) {
    state_.velocitySample = sample;
    state_.velocitySeen = true;
  }
  giveState();
  return result == robofish::control::l1::SampleValidation::Valid;
}

bool MotionController::getFeedback(robofish::ControlFeedback &snapshot,
    uint32_t maximumAgeMs) const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  snapshot = robofish::control::l1::feedbackSnapshot(
      state_.diagnostics.attitudeAvailable ? &state_.diagnostics.attitude : nullptr,
      state_.depthSeen ? &state_.depthSample : nullptr,
      state_.velocitySeen ? &state_.velocitySample : nullptr,
      nowMs(), maximumAgeMs);
  giveState();
  return true;
}

bool MotionController::clearFeedback() {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  state_.diagnostics.attitudeAvailable = false;
  state_.depthSeen = false;
  state_.velocitySeen = false;
  giveState();
  return true;
}
