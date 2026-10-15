#pragma once

#include "bsp/ServoPwm.h"
#include "DeviceMutex.h"

class PectoralFins {
 public:
  bool begin();

  bool setBoth(int leftAngle, int rightAngle, TickType_t timeout);
  uint8_t setBothMask(int leftAngle, int rightAngle, TickType_t timeout);

 private:
  ServoPwm leftServo_;
  ServoPwm rightServo_;
  DeviceMutex mutex_;
};
