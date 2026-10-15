#include <atomic>
#include <cassert>
#include <cstdio>
#define private public
#include "task/MotionController.h"
#undef private
int main() {
  RoboFishDriver driver;
  MotionController controller(driver);
  assert(controller.begin());
  auto tick=[&](uint32_t now) {mockTick=now;controller.publishSpiLink(now,true);controller.controlTick(now);};
  auto sample=[&](uint16_t raw,uint16_t display,bool valid=true) {
    robofish::BatterySample v{1,mockTick,display,3300,1000,valid,raw};
    assert(controller.publishBatterySample(v));tick(mockTick);
  };
  const robofish::lora::Command forward{1,2,true,1000};
  tick(1000);
  assert(!controller.submitLoRa(forward));
  sample(7400,7400);
  assert(controller.submitLoRa(forward));
  for (uint8_t code : {uint8_t(8),uint8_t(18),uint8_t(19)}) {
    assert(!controller.submitLoRa({2,code,true,0}));
    assert(controller.state_.remote.target.knob==1000);
  }
  for(uint32_t now=1020;now<=1200;now+=20) tick(now);
  assert(driver.last.leftPropulsionPercent || driver.last.rightPropulsionPercent);
  auto stopped=[&] {
    assert(driver.last.leftPropulsionPercent==0 && driver.last.rightPropulsionPercent==0);
    assert(driver.last.tailFrontDegrees==90 && driver.last.tailRearDegrees==90);
    assert(driver.last.pectoralLeftDegrees==90 && driver.last.pectoralRightDegrees==90);
  };
  sample(4,7400); // A healthy filtered value must not hide a raw low reading.
  stopped();assert(!controller.submitLoRa(forward));
  assert(!controller.submitManual({MotionAction::Forward,50,50}));
  assert(!controller.applyAutomationPitch(100));
  assert(controller.applyVisionControl({1,20,0,false})==VisionControlResult::Rejected);
  assert(controller.applyVisionPid(1,0,1,0,0,false)==VisionControlResult::Rejected);
  sample(7400,7400);tick(1220);stopped(); // Recovery never restores the old request.
  assert(controller.submitLoRa({2,2,true,1000}));
  for(uint32_t now=1240;now<=1400;now+=20)tick(now);
  assert(driver.last.leftPropulsionPercent || driver.last.rightPropulsionPercent);
  sample(8401,7400);stopped();
  sample(7400,7400,false);stopped();
  sample(7400,7400);stopped();
  assert(controller.submitLoRa({3,2,true,1000}));tick(1420);
  tick(1901);stopped(); // Stale sample also stops.
  // A failed neutral submission must complete before a new target may run.
  sample(7400,7400);assert(controller.submitLoRa({4,2,true,1000}));tick(1920);
  driver.writtenMask=0;sample(6500,7400);
  sample(7400,7400);assert(controller.submitLoRa({5,2,true,1000}));tick(1940);
  // Hardware rejected the write: prove the requested frame is neutral, not that
  // an unwritten frame physically stopped the actuators.
  assert(driver.attempted.leftPropulsionPercent==0 && driver.attempted.rightPropulsionPercent==0);
  driver.writtenMask=robofish::AllFrameChannels;tick(1960);stopped();
  puts("PASS: raw-voltage stop, invalid/stale stop, all six neutral, rejected commands, recovery requires new input, failed-stop barrier");
}
