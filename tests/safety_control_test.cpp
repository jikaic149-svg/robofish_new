#include <atomic>
#include <cassert>
#include <cstdio>
#define private public
#include "task/MotionController.h"
#undef private
#include "bsp/RuntimeSafety.h"

using robofish::SafetyMode;
struct Rig {
  RoboFishDriver driver;
  MotionController control{driver};
  uint16_t sequence=0;
  bool peer=true;
  Rig() { assert(control.begin()); }
  void tick(uint32_t now, uint16_t volts=7400, bool radioTask=true) {
    mockTick=now;
    if (peer) control.publishSpiLink(now,true);
    assert(control.publishBatterySample({1,now,volts,3300,1000,true,volts}));
    if (radioTask) control.noteRadioCycle(now);
    const bool done=control.controlTick(now);
    control.serviceHealth(now,done);
  }
  bool cmd(uint8_t code) { return control.submitLoRa({++sequence,code,true,1000}); }
  SafetyMode mode() const {return control.state_.safety.mode;}
  void neutral() const {
    assert(driver.last.leftPropulsionPercent==0 && driver.last.rightPropulsionPercent==0);
    assert(driver.last.tailFrontDegrees==90 && driver.last.tailRearDegrees==90);
    assert(driver.last.pectoralLeftDegrees==90 && driver.last.pectoralRightDegrees==90);
  }
};
static void generations() {
  Rig v;v.tick(0);
  assert(v.control.setMode(DriveMode::Hybrid));
  assert(v.control.submitManual({MotionAction::Forward,40,40}));v.tick(20);
  const auto epoch=v.control.state_.safety.generation;
  VisionSetpoint vision{1,20,0,false,epoch};
  assert(v.control.applyVisionControl(vision)==VisionControlResult::Accepted);
  assert(v.cmd(17));v.tick(40);v.neutral();
  assert(v.control.applyVisionControl(vision)==VisionControlResult::Rejected);
  assert(!v.control.submitAutomation({MotionAction::Forward,40,40},epoch));
  assert(v.control.setMode(DriveMode::Hybrid));
  assert(v.control.submitManual({MotionAction::Forward,40,40}));v.tick(60);
  assert(v.control.applyVisionControl(vision)==VisionControlResult::Rejected);
  assert(!v.control.automationSafeStop(epoch)); // Old completion cannot stop a new operator goal.
  vision.controlGeneration=v.control.state_.safety.generation;
  assert(v.control.applyVisionControl(vision)==VisionControlResult::Accepted);
  assert(v.control.automationSafeStop(vision.controlGeneration));v.tick(80);v.neutral();
  assert(v.control.applyVisionControl(vision)==VisionControlResult::Rejected);
}
static void recoveryLockFailure() {
  Rig v;v.tick(100);assert(v.cmd(2));v.tick(120);
  v.driver.writtenMask=0;
  for(unsigned i=1;i<5;++i)v.tick(120+20*i);
  v.driver.context=&v.control;
  v.driver.afterAttempt=[](void *ctx){static_cast<MotionController *>(ctx)->stateMutex_->locked=true;};
  v.tick(220);assert(v.control.outputRecovery_);
  v.driver.afterAttempt=nullptr;v.control.stateMutex_->locked=false;
  v.driver.writtenMask=robofish::AllFrameChannels;
  v.tick(240);v.neutral();assert(v.mode()==SafetyMode::WaitCommand);
  v.tick(260);v.neutral(); // Old target must not return after recovery clears.
}
int main() {
  using namespace robofish::bsp;
  mockFault=false;
  robofish::SafetyPolicy wrap;
  assert(wrap.motion(0xFFFFFF00U,robofish::ControlOwner::LoRa,true,true));
  assert(!wrap.update(uint32_t(0xFFFFFF00U+2999U),true,3000));
  assert(wrap.update(uint32_t(0xFFFFFF00U+3000U),true,3000));
  assert(wrap.mode==SafetyMode::Surface);
  generations();
  recoveryLockFailure();
  Rig r;
  r.peer=false; r.tick(1000);
  assert(!r.cmd(2)); // Startup blocks without SPI even with a valid LoRa command.
  r.peer=true; r.tick(1020); assert(r.cmd(2));
  for(uint32_t t=1040;t<1200;t+=20)r.tick(t);
  assert(r.driver.last.leftPropulsionPercent!=0);
  r.peer=false;
  for(uint32_t t=1200;t<4020;t+=20)r.tick(t);
  assert(r.mode()==SafetyMode::Running); // SPI loss cannot preempt healthy LoRa lease.
  auto held=r.driver.last;
  const auto feeds=mockFeeds;
  r.control.observeLoRa({}); // Diagnostic publication is not command renewal.
  r.tick(4020);
  assert(r.mode()==SafetyMode::Surface);
  for(uint32_t t=4040;t<4200;t+=20) {
    r.tick(t);
    assert(r.driver.last.leftPropulsionPercent==0 && r.driver.last.rightPropulsionPercent==0);
    assert(r.driver.last.tailFrontDegrees==held.tailFrontDegrees);
    assert(r.driver.last.tailRearDegrees==held.tailRearDegrees);
    assert(r.driver.last.pectoralLeftDegrees==held.pectoralLeftDegrees);
    assert(r.driver.last.pectoralRightDegrees==held.pectoralRightDegrees);
  }
  assert(mockFeeds>feeds && mockResets==0); // SURFACE feeds normally.
  assert(r.cmd(14));r.tick(4200);assert(r.mode()==SafetyMode::Surface);
  assert(r.cmd(17));r.tick(4220);r.neutral();
  r.tick(8000);assert(r.mode()==SafetyMode::WaitCommand);r.neutral();
  assert(!r.cmd(2)); // Stopped + SPI down cannot restart.
  r.peer=true;r.tick(8020);assert(r.cmd(5));r.tick(8040);r.neutral();
  assert(r.mode()==SafetyMode::WaitCommand); // Auxiliary command cannot rearm.
  assert(r.cmd(2));r.tick(8060);
  r.tick(8080,6500);assert(r.mode()==SafetyMode::BatteryStop);r.neutral();
  auto batteryFeeds=mockFeeds;
  r.tick(8100,6500);assert(mockFeeds>batteryFeeds);
  r.tick(8120);assert(r.mode()==SafetyMode::WaitCommand);r.neutral();
  assert(r.cmd(14));r.tick(8140);r.neutral();
  assert(r.cmd(2));r.tick(8160);

  // A transient write error is a warning and does not suppress health feeding.
  r.driver.writtenMask=0;
  auto failFeeds=mockFeeds;
  r.tick(8180);r.tick(8200);
  assert(mockFeeds==failFeeds+2 && r.mode()==SafetyMode::Running);
  r.driver.writtenMask=robofish::AllFrameChannels;r.tick(8220);
  assert(r.control.consecutiveWriteFailures_==0);
  r.driver.writtenMask=0;
  for(unsigned i=1;i<=5;i++)r.tick(8220+20*i);
  assert(r.control.outputRecovery_ && r.mode()==SafetyMode::WaitCommand);
  assert(!r.cmd(2));
  r.driver.writtenMask=robofish::AllFrameChannels;r.tick(8340);r.neutral();
  assert(!r.control.outputRecovery_);r.tick(8360);r.neutral();
  assert(r.cmd(2));r.tick(8380);

  // Task health uses individual elapsed deadlines, including counter wrap.
  robofish::TaskHealth health;
  assert(!health.healthy(0,100));health.complete(0xFFFFFFF0U);
  assert(health.healthy(0x10U,100));assert(!health.healthy(0x80U,100));
  auto oldFeeds=mockFeeds;
  r.tick(8500,7400,false);assert(mockFeeds==oldFeeds); // Radio task stalled, not RF silent.
  r.tick(8520);assert(mockFeeds==oldFeeds+1);

  // Persistent failed outputs escalate while healthy cycles continue feeding.
  r.driver.writtenMask=0;
  failFeeds=mockFeeds;
  for(unsigned i=1;i<15;i++)r.tick(8520+20*i);
  assert(mockFeeds==failFeeds+14 && mockResets==0);
  r.tick(8820);
  assert(mockResets==0 && mockFault && mockControlled && mockReason==FaultReason::OutputWriteFailure);
  assert(mockFeeds==failFeeds+15 && mockEmergencyWrites==1);
  auto attempts=r.driver.attempts;
  r.tick(8840);assert(r.driver.attempts==attempts && !r.cmd(2));
  assert(mockResets==0 && mockFeeds==failFeeds+16);
  r.tick(8860);assert(mockResets==1 && mockFeeds==failFeeds+16);
  puts("PASS: startup gate, LoRa lease, fixed-fin zero-thrust surface, stop/battery barriers, warning/recovery/fatal ladder, watchdog independence and wrap");
}
