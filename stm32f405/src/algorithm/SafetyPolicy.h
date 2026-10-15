#pragma once
#include <stdint.h>

namespace robofish {
enum class SafetyMode : uint8_t { WaitCommand, Running, Surface, BatteryStop };
enum class ControlOwner : uint8_t { None, LoRa, Local };
enum StartBlock : uint32_t {
  StartBattery = 1U, StartSpiOffline = 2U, StartNewCommand = 4U,
  StartFault = 8U, StartOutputRecovery = 16U, StartBootTrial = 32U,
};

// Pure policy. A link becoming healthy never authorizes an old target.
struct SafetyPolicy {
  SafetyMode mode = SafetyMode::WaitCommand;
  ControlOwner owner = ControlOwner::None;
  uint32_t lastTargetMs = 0;
  uint32_t generation = 0;

  void exit(SafetyMode next = SafetyMode::WaitCommand) {
    mode = next; owner = ControlOwner::None; ++generation;
  }
  bool motion(uint32_t now, ControlOwner source, bool battery, bool spi,
              bool blocked = false) {
    if (!battery || blocked) return false;
    if ((mode == SafetyMode::WaitCommand || mode == SafetyMode::BatteryStop) && !spi)
      return false;
    mode = SafetyMode::Running; owner = source; lastTargetMs = now; ++generation;
    return true;
  }
  void renewLoRa(uint32_t now) {
    if (mode == SafetyMode::Running && owner == ControlOwner::LoRa) lastTargetMs = now;
  }
  bool update(uint32_t now, bool battery, uint32_t leaseMs) {
    const auto before = mode;
    if (!battery) {
      if (mode != SafetyMode::BatteryStop) exit(SafetyMode::BatteryStop);
    } else if (mode == SafetyMode::BatteryStop) {
      exit();
    } else if (mode == SafetyMode::Running && owner == ControlOwner::LoRa &&
               leaseMs && uint32_t(now - lastTargetMs) >= leaseMs) {
      exit(SafetyMode::Surface);
    }
    return mode != before;
  }
};
inline bool isLoRaMotion(uint8_t code) {
  return (code >= 2 && code <= 4) || code == 6 || code == 7 ||
         (code >= 9 && code <= 13);
}
}
