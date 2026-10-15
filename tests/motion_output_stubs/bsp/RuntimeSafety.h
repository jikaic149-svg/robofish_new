#pragma once

// Inject an ISR-latched fault between real ServoPwm submissions, without
// replacing RoboFishDriver, the individual devices or the PWM BSP.
namespace robofish::bsp {
inline unsigned outputAttempt = 0;
inline unsigned latchAtAttempt = 0;
inline bool faultLatched() {
  ++outputAttempt;
  return latchAtAttempt && outputAttempt >= latchAtAttempt;
}
}
