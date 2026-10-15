#pragma once
#include <stdint.h>
#include "config/ControlConfig.h"
namespace robofish {
// RadioPoll publishes one latest sample; MotionControl consumes it at 20 ms.
struct BatterySample {
  uint32_t sequence = 0;
  uint32_t sampledAtMs = 0;
  uint16_t millivolts = 0;
  uint16_t vddaMillivolts = 0;
  uint16_t adcCounts = 0;
  bool valid = false;  // ADC conversion valid, independent of operating limits.
  uint16_t rawMillivolts = 0;  // Before display filtering; used by the interlock.
};
enum class BatteryCondition : uint8_t { Normal = 0, Low = 1, High = 2, Unavailable = 3 };
inline BatteryCondition batteryCondition(const BatterySample &sample,
    uint32_t nowMs, uint32_t maximumAgeMs) {
  if (!sample.valid || nowMs - sample.sampledAtMs > maximumAgeMs)
    return BatteryCondition::Unavailable;
  if (sample.rawMillivolts < config::battery::kMinimumOperatingMv)
    return BatteryCondition::Low;
  if (sample.rawMillivolts > config::battery::kMaximumOperatingMv)
    return BatteryCondition::High;
  return BatteryCondition::Normal;
}
inline uint16_t usableBatteryMillivolts(const BatterySample &sample,
    uint32_t nowMs, uint32_t maximumAgeMs) {
  return batteryCondition(sample, nowMs, maximumAgeMs) == BatteryCondition::Normal &&
      sample.millivolts >= config::battery::kMinimumOperatingMv &&
      sample.millivolts <= config::battery::kMaximumOperatingMv ? sample.millivolts : 0;
}
}
