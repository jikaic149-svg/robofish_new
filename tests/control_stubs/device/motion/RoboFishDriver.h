#pragma once
#include "channel/FrameApplyResult.h"
#include "device/motion/EscCalibration.h"
#include "bsp/RuntimeSafety.h"
class RoboFishDriver {
 public:
  robofish::ActuatorFrame last;
  robofish::ActuatorFrame attempted;
  uint8_t writtenMask=robofish::AllFrameChannels;
  uint32_t attempts=0;
  void (*afterAttempt)(void *)=nullptr;
  void *context=nullptr;
  bool ready() const {return true;}
  robofish::FrameApplyResult update() {return {};}
  robofish::FrameApplyResult applyFrameDetailed(const robofish::ActuatorFrame &frame) {
    if (robofish::bsp::faultLatched()) return {};
    ++attempts;
    attempted=frame;
    auto actual=frame;
    actual.leftPropulsionPercent=robofish::device::propulsionElectricalPercent(frame.leftPropulsionPercent,robofish::device::PropulsionSide::Left,frame.supplyMillivolts);
    actual.rightPropulsionPercent=robofish::device::propulsionElectricalPercent(frame.rightPropulsionPercent,robofish::device::PropulsionSide::Right,frame.supplyMillivolts);
    robofish::mergeWrittenFrame(last,actual,writtenMask);
    if(afterAttempt)afterAttempt(context);
    return {writtenMask,int16_t(actual.leftPropulsionPercent),int16_t(actual.rightPropulsionPercent)};
  }
};
