#pragma once

#include <stdint.h>

#include "bsp/BoardResources.h"

class ServoPwm {
 public:
  ServoPwm();
  static bool configureTimers();
  bool begin(robofish::bsp::BoardPin gpio, uint32_t channel,
             robofish::bsp::PwmTimer timer, int minPulseUs, int maxPulseUs);
  bool writeAngle(int angle);
  bool writeMicroseconds(int pulseUs);

 private:
  void *timer_ = nullptr;
  uint32_t channel_ = 0;
  int minPulseUs_ = 0;
  int maxPulseUs_ = 0;
  bool initialized_ = false;
};
