#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#define private public
#include "task/MotionController.h"
#undef private
#include "config/MotionAllocationConfig.h"

// Exercise the real task pipeline, with only its RTOS/physical writer replaced.
struct Rig {
  RoboFishDriver driver;
  MotionController control{driver};
  uint32_t time = 100;
  uint16_t sequence = 0;
  Rig() { assert(control.begin()); tick(); }
  void tick() {
    mockTick = time;
    control.publishSpiLink(time, true);
    assert(control.publishBatterySample({1,time,7400,3300,1000,true,7400}));
    assert(control.controlTick(time));
    time += 20;
  }
  void run(unsigned count) { while (count--) tick(); }
  void remote(uint8_t code) { assert(control.submitLoRa({++sequence,code,true,1000})); }
  void neutral() const {
    assert(driver.last.leftPropulsionPercent == 0 && driver.last.rightPropulsionPercent == 0);
    assert(driver.last.tailFrontDegrees == 90 && driver.last.tailRearDegrees == 90);
    assert(driver.last.pectoralLeftDegrees == 90 && driver.last.pectoralRightDegrees == 90);
  }
};

static void reverseCannotResurrect() {
  Rig r;
  r.remote(15); r.remote(2); r.run(5);
  r.remote(9); r.tick();
  assert(r.control.state_.safety.mode == robofish::SafetyMode::WaitCommand);
  assert(r.control.state_.diagnostics.limitedReasons & robofish::control::l2::ReverseUnsupported);
  r.neutral();
  r.remote(16); r.run(2); r.neutral(); // Mode change alone cannot restore forward motion.
  r.remote(9); r.run(15);
  assert(r.driver.last.leftPropulsionPercent < 0 && r.driver.last.rightPropulsionPercent > 0);
  assert(r.driver.last.tailFrontDegrees == 90 && r.driver.last.tailRearDegrees == 90);
}

static void flowStopAndPartialHistory() {
  Rig r;
  r.remote(16); r.remote(6); r.run(12);
  assert(r.driver.attempted.flowAssisted);
  assert(r.driver.last.leftPropulsionPercent > 0 && r.driver.last.rightPropulsionPercent < 0);
  const auto fins = r.driver.last;
  r.remote(1); r.run(12);
  assert(r.driver.last.leftPropulsionPercent == 0 && r.driver.last.rightPropulsionPercent == 0);
  assert(!r.driver.attempted.flowAssisted);
  assert(r.driver.last.pectoralLeftDegrees == fins.pectoralLeftDegrees);
  r.remote(2); r.run(2);
  const auto before = r.control.committed_;
  r.driver.writtenMask = robofish::TailFront | robofish::PropulsionLeft;
  r.tick();
  assert(r.control.committed_.tailRearDegrees == before.tailRearDegrees);
  assert(r.control.committed_.rightPropulsionPercent == before.rightPropulsionPercent);
  assert(r.control.committed_.leftPropulsionPercent == r.driver.attempted.leftPropulsionPercent);
  assert(r.control.committed_.rightPropulsionPercent >= 0); // Never feeds electrical polarity back.
  r.driver.writtenMask = robofish::AllFrameChannels;
  r.remote(17); r.tick(); r.neutral();
}

static void sameHardwareAndIndependentVision() {
  Rig remote;
  remote.remote(14); remote.remote(2); remote.run(12);
  Rig manual;
  assert(manual.control.setMode(DriveMode::Prop));
  assert(manual.control.submitManual({MotionAction::Forward,100,0}));
  manual.run(12);
  assert(remote.driver.last.tailFrontDegrees == manual.driver.last.tailFrontDegrees);
  assert(remote.driver.last.pectoralLeftDegrees == manual.driver.last.pectoralLeftDegrees);
  assert(remote.driver.last.leftPropulsionPercent == manual.driver.last.leftPropulsionPercent);
  assert(remote.driver.last.rightPropulsionPercent == manual.driver.last.rightPropulsionPercent);

  assert(manual.control.setMode(DriveMode::Hybrid));
  assert(manual.control.submitManual({MotionAction::Forward,50,0}));
  const auto generation = manual.control.state_.safety.generation;
  assert(manual.control.applyVisionPid(1,0,0.05f,0,0,false,generation) == VisionControlResult::Accepted);
  assert(manual.control.visionTick(manual.time));
  assert(manual.control.state_.vision.tailPowerPercent != manual.control.state_.vision.propulsionPowerPercent);
  manual.run(12);
  assert(manual.control.state_.diagnostics.commonPropulsionPercent ==
      manual.control.state_.vision.propulsionPowerPercent);
}

static void algorithmsUseTheSamePipeline() {
  Rig r;
  robofish::control::l1::BodyCommand body{35,12,40,25,true};
  assert(!r.control.submitBodyCommand(DriveMode::Hybrid,body,0));
  assert(r.control.setMode(DriveMode::Hybrid));
  assert(r.control.submitManual({MotionAction::Forward,30,0}));
  const auto generation = r.control.state_.safety.generation;
  assert(r.control.submitBodyCommand(DriveMode::Hybrid,body,generation));
  r.run(10);
  assert(r.control.state_.diagnostics.lastAttemptSource == MotionSource::Algorithm);
  assert(r.control.committed_.leftPropulsionPercent > r.control.committed_.rightPropulsionPercent);
  assert(r.control.committed_.pectoralLeftDegrees > 90);
  auto invalid = body;
  invalid.rollPercent = std::numeric_limits<float>::quiet_NaN();
  assert(!r.control.submitBodyCommand(DriveMode::Hybrid,invalid,generation));
  assert(r.control.state_.bodyTarget.rollPercent == 25);
  assert(r.control.setPitch(RF_PITCH_CENTER));
  r.run(12);
  assert(r.control.committed_.leftPropulsionPercent == 0);
  assert(r.control.committed_.rightPropulsionPercent == 0); // No dormant manual goal.
  assert(r.control.submitManual({MotionAction::Stopped,0,0}));
  assert(!r.control.submitBodyCommand(DriveMode::Hybrid,body,generation));
  r.tick(); r.neutral();
  r.remote(2);
  assert(!r.control.submitBodyCommand(DriveMode::Hybrid,body,r.control.state_.safety.generation));
}

int main() {
  reverseCannotResurrect();
  flowStopAndPartialHistory();
  sameHardwareAndIndependentVision();
  algorithmsUseTheSamePipeline();
  std::puts("PASS: L0-L3 task integration, reverse withdrawal, flow STOP, partial nominal history and independent visual drives");
}
