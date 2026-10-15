#pragma once
#include <atomic>
#include <FreeRTOS.h>
#include "channel/FrameApplyResult.h"
#include "Propulsion.h"
#include "PectoralFins.h"
#include "TailServos.h"

// L3: six physical outputs, electrical calibration and per-channel submission.
// Propulsion inputs are nominal forward-positive percentages on BOTH sides;
// installed polarity and voltage compensation are applied at this boundary.
// Servo angles retain the existing calibrated command-angle coordinates.
class RoboFishDriver {
 public:
  bool begin();
  robofish::FrameApplyResult update();
  robofish::FrameApplyResult applyFrameDetailed(const robofish::ActuatorFrame &frame);
  bool ready() const;

 private:
  bool stopAllOutputs(TickType_t timeout);
  TailServos tail_;
  PectoralFins pectoral_;
  Propulsion propulsion_;
  std::atomic_bool ready_{false};
};
