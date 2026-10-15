#pragma once

#include "bsp/ServoPwm.h"
#include "DeviceMutex.h"

class TailServos {
 public:
  bool begin();

  bool applyBoth(int angle1, int angle2, TickType_t timeout);
  uint8_t applyBothMask(int angle1, int angle2, TickType_t timeout);

 private:

  ServoPwm servo1_;
  ServoPwm servo2_;
  DeviceMutex mutex_;
};
