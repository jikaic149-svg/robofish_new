#include "TailServos.h"

#include <algorithm>

#include "bsp/BoardResources.h"
#include "config/RuntimeConfig.h"

bool TailServos::begin() {
  if (!mutex_.begin()) return false;
  bool attached = true;
  const auto &servo1 = robofish::bsp::kTailServo1;
  const auto &servo2 = robofish::bsp::kTailServo2;
  attached &= servo1_.begin(servo1.gpio, servo1.channel, servo1.timer,
                            servo1.minimumPulseUs, servo1.maximumPulseUs);
  attached &= servo2_.begin(servo2.gpio, servo2.channel, servo2.timer,
                            servo2.minimumPulseUs, servo2.maximumPulseUs);
  return attached;
}

bool TailServos::applyBoth(int angle1, int angle2, TickType_t timeout) {
  return applyBothMask(angle1, angle2, timeout) == 3;
}

uint8_t TailServos::applyBothMask(int angle1, int angle2, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock) return false;
  const bool first = servo1_.writeAngle(std::clamp(angle1, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
  const bool second = servo2_.writeAngle(std::clamp(angle2, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
  return (first ? 1 : 0) | (second ? 2 : 0);
}
