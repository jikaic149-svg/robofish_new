#pragma once

#include <stdint.h>

namespace robofish::bsp {

// These are logical board pins rather than STM32 HAL GPIO constants.  The
// numeric values exist solely for existing diagnostics that print `gpio`.
enum BoardPin : int {
  kNotConnected = -1,
  kPa0 = 0,
  kPa1 = 1,
  kPa8 = 8,
  kPa15 = 15,
  kPb0 = 16,
  kPb1 = 17,
  kPb8 = 24,
  kPa9 = 9,
  kPa10 = 10,
  kPc2 = 34,
};

enum class PwmTimer : uint8_t { kTim1, kTim2, kTim3, kTim5 };

struct ServoResource {
  BoardPin gpio;
  uint32_t channel;
  PwmTimer timer;
  int minimumPulseUs;
  int maximumPulseUs;
};

// EDA PCB1: U13 pins 7/6/5/2/3/4. TIM5 leaves TIM2_CH1 independent on PA15.
// U13 -> distribution-board U7 uses an eight-core, pin-for-pin harness.
// PCB3: FRONT2/CN5 = left fin, FRONT1/CN8 = right fin,
// rear upper/CN7 = tail 1, rear lower/CN6 = tail 2.
inline constexpr ServoResource kTailServo1 = {BoardPin::kPa1, 2, PwmTimer::kTim5, 500, 2500};
inline constexpr ServoResource kTailServo2 = {BoardPin::kPa15, 1, PwmTimer::kTim2, 500, 2500};
inline constexpr ServoResource kPectoralLeftServo = {BoardPin::kPa8, 1, PwmTimer::kTim1, 500, 2500};
inline constexpr ServoResource kPectoralRightServo = {BoardPin::kPa0, 1, PwmTimer::kTim5, 500, 2500};
inline constexpr ServoResource kPropulsionLeft = {BoardPin::kPb0, 3, PwmTimer::kTim3, 1000, 2000};
inline constexpr ServoResource kPropulsionRight = {BoardPin::kPb1, 4, PwmTimer::kTim3, 1000, 2000};

inline constexpr BoardPin kLoRaTxGpio = BoardPin::kPa9;
inline constexpr BoardPin kLoRaRxGpio = BoardPin::kPa10;
inline constexpr BoardPin kRgbDataGpio = BoardPin::kPb8;
// U7.8 -> U13.8 -> PC2 / ADC1_IN12, divider R3 (VBAT) / R2 (GND).
inline constexpr BoardPin kBatterySenseGpio = BoardPin::kPc2;
inline constexpr uint32_t kBatteryDividerUpperOhms = 10000;
inline constexpr uint32_t kBatteryDividerLowerOhms = 2200;

}  // namespace robofish::bsp
