#pragma once

#include <stdint.h>

#include "bsp/ServoPwm.h"
#include "DeviceMutex.h"

class Propulsion {
 public:
  bool begin();

  uint8_t setDifferentialMask(int leftPercent, int rightPercent, TickType_t timeout);
  bool stop(TickType_t timeout);
  uint8_t updateMask(uint32_t nowMs, TickType_t timeout);

 private:
  uint8_t stopUnlocked();

  ServoPwm left_;
  ServoPwm right_;
  DeviceMutex mutex_;
  int leftPercent_ = 0;
  int rightPercent_ = 0;
  uint32_t leftCommandMs_ = 0, rightCommandMs_ = 0;
  bool leftHasCommand_ = false, rightHasCommand_ = false;
};
