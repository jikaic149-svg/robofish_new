#include "Propulsion.h"

#include <algorithm>

#include "bsp/BoardResources.h"
#include "config/RuntimeConfig.h"
#include "bsp/Time.h"
#include "device/motion/EscCalibration.h"

bool Propulsion::begin() {
  if (!mutex_.begin()) return false;
  bool attached = true;
  const auto &left = robofish::bsp::kPropulsionLeft;
  const auto &right = robofish::bsp::kPropulsionRight;
  attached &= left_.begin(left.gpio, left.channel, left.timer,
                          left.minimumPulseUs, left.maximumPulseUs);
  attached &= right_.begin(right.gpio, right.channel, right.timer,
                           right.minimumPulseUs, right.maximumPulseUs);
  return attached;
}

uint8_t Propulsion::setDifferentialMask(int leftPercent, int rightPercent, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock) return false;
  const bool left = left_.writeMicroseconds(robofish::device::propulsionPulseUs(leftPercent));
  const bool right = right_.writeMicroseconds(robofish::device::propulsionPulseUs(rightPercent));
  const uint32_t now = robofish::bsp::nowMs();
  if (left) { leftPercent_ = std::clamp(leftPercent, -100, 100); leftCommandMs_ = now; leftHasCommand_ = true; }
  if (right) { rightPercent_ = std::clamp(rightPercent, -100, 100); rightCommandMs_ = now; rightHasCommand_ = true; }
  return (left ? 1 : 0) | (right ? 2 : 0);
}

bool Propulsion::stop(TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock) return false;
  return stopUnlocked() == 3;
}

uint8_t Propulsion::updateMask(uint32_t nowMs, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock) return false;

  uint8_t stopped = 0;
  if (leftPercent_ != 0 && leftHasCommand_ && nowMs - leftCommandMs_ >= RF_PROPULSION_TIMEOUT_MS) {
    if (left_.writeMicroseconds(robofish::config::lora::kEscNeutralUs)) {
      leftPercent_ = 0; leftHasCommand_ = false; stopped |= 1;
    }
  }
  if (rightPercent_ != 0 && rightHasCommand_ && nowMs - rightCommandMs_ >= RF_PROPULSION_TIMEOUT_MS) {
    if (right_.writeMicroseconds(robofish::config::lora::kEscNeutralUs)) {
      rightPercent_ = 0; rightHasCommand_ = false; stopped |= 2;
    }
  }
  return stopped;
}

uint8_t Propulsion::stopUnlocked() {
  const bool left = left_.writeMicroseconds(robofish::config::lora::kEscNeutralUs);
  const bool right = right_.writeMicroseconds(robofish::config::lora::kEscNeutralUs);
  if (left) { leftPercent_ = 0; leftHasCommand_ = false; }
  if (right) { rightPercent_ = 0; rightHasCommand_ = false; }
  return (left ? 1 : 0) | (right ? 2 : 0);
}
