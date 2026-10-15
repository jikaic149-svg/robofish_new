#pragma once
#include "channel/MotionTypes.h"
#include "channel/MotionDiagnostics.h"
class MotionController {
 public:
  void publishSpiLink(uint32_t, bool) {}
  bool getStatus(MotionStatus&) {return false;}
  bool getDiagnostics(MotionDiagnostics&) {return false;}
};
