#include "PectoralFins.h"

#include <algorithm>

#include "bsp/BoardResources.h"
#include "config/RuntimeConfig.h"

bool PectoralFins::begin() {
  if (!mutex_.begin()) return false;
  const auto &servo = robofish::bsp::kPectoralLeftServo;
  bool ready = leftServo_.begin(servo.gpio, servo.channel, servo.timer,
                           servo.minimumPulseUs, servo.maximumPulseUs);
  const auto &right = robofish::bsp::kPectoralRightServo;
  ready &= rightServo_.begin(right.gpio, right.channel, right.timer,
                            right.minimumPulseUs, right.maximumPulseUs);
  return ready;
}

bool PectoralFins::setBoth(int leftAngle, int rightAngle, TickType_t timeout) {
  return setBothMask(leftAngle, rightAngle, timeout) == 3;
}

uint8_t PectoralFins::setBothMask(int leftAngle, int rightAngle, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock) return false;
  namespace calibration = robofish::config::lora;
  const bool left = leftServo_.writeAngle(std::clamp(leftAngle, calibration::kPectoralLeftMinimum,
                                             calibration::kPectoralLeftMaximum));
  const bool right = rightServo_.writeAngle(std::clamp(rightAngle, calibration::kPectoralRightMinimum,
                                              calibration::kPectoralRightMaximum));
  return (left ? 1 : 0) | (right ? 2 : 0);
}
