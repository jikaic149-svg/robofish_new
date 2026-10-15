#pragma once

#include <algorithm>
#include <stdint.h>

#include "config/RuntimeConfig.h"
#include "config/ControlConfig.h"

namespace robofish::device {

// Use the current signed command on every call, never a previous compensated output.
// Invalid/out-of-range voltage or reference is neutral; zero stays neutral.
inline int voltageCompensatedPropulsionPercent(int commandPercent,
    uint16_t referenceMv, uint16_t measuredMv) {
  commandPercent = std::clamp(commandPercent, -100, 100);
  if (!referenceMv || measuredMv < config::battery::kMinimumOperatingMv ||
      measuredMv > config::battery::kMaximumOperatingMv || !commandPercent) return 0;
  const bool reversed = commandPercent < 0;
  const uint32_t magnitude = uint32_t(reversed ? -commandPercent : commandPercent);
  const uint32_t scaled = (magnitude * referenceMv + measuredMv / 2U) / measuredMv;
  const int compensated = int(std::min<uint32_t>(scaled, 100U));
  return reversed ? -compensated : compensated;
}

enum class PropulsionSide : uint8_t { Left, Right };

// Shared L3 mapping for the physical writer and the PC target preview. Both
// inputs use forward-positive nominal commands. Clamp before installation
// inversion so INT_MIN cannot overflow when the configured sign is negative.
inline int propulsionElectricalPercent(int nominalPercent, PropulsionSide side,
    uint16_t measuredMv) {
  const int direction = side == PropulsionSide::Left ?
      config::lora::kLeftEscDirection : config::lora::kRightEscDirection;
  return voltageCompensatedPropulsionPercent(
      std::clamp(nominalPercent, -100, 100) * direction,
      config::propulsion::kReferenceVoltageMv, measuredMv);
}

// 双向 ESC 的电气标定属于 Device，不由运动策略层操作 PWM。
inline int propulsionPulseUs(int percent) {
  namespace calibration = robofish::config::lora;
  percent = std::clamp(percent, -100, 100);
  return calibration::kEscMinimumUs + (percent + 100) *
      (calibration::kEscMaximumUs - calibration::kEscMinimumUs) / 200;
}

}  // namespace robofish::device
