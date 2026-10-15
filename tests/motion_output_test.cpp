#include <cassert>
#include <climits>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

#include "main.h"
#include "bsp/RuntimeSafety.h"
#include "device/motion/RoboFishDriver.h"
#include "channel/FrameApplyResult.h"

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5;
UART_HandleTypeDef huart1;

namespace {
void resetTimers() {
  htim1 = {}; htim2 = {}; htim3 = {}; htim4 = {}; htim5 = {};
  htim1.Instance = TIM1; htim2.Instance = TIM2; htim3.Instance = TIM3;
  htim4.Instance = TIM4; htim5.Instance = TIM5;
  mockTick = 0;
  pwmResult = HAL_OK;
  robofish::bsp::outputAttempt = robofish::bsp::latchAtAttempt = 0;
}
robofish::ActuatorFrame nominal(int left = 50, int right = 50) {
  robofish::ActuatorFrame frame;
  frame.leftPropulsionPercent = left;
  frame.rightPropulsionPercent = right;
  frame.supplyMillivolts = 6800;
  return frame;
}
void checkEscPulse(const robofish::FrameApplyResult &applied, int left, int right) {
  assert(applied.leftPropulsionPercent == left);
  assert(applied.rightPropulsionPercent == right);
  assert(htim3.compare[2] == unsigned(1500 + left * 5));
  assert(htim3.compare[3] == unsigned(1500 + right * 5));
}
}

int main() {
  using namespace robofish;
  resetTimers();
  RoboFishDriver unready;
  assert(!unready.ready());
  assert(unready.applyFrameDetailed(nominal()).writtenMask == 0);
  assert(unready.update().writtenMask == 0);
  pwmResult = HAL_ERROR;
  assert(!unready.begin() && !unready.ready());

  resetTimers();
  RoboFishDriver driver;
  assert(driver.begin() && driver.ready());
  assert(htim5.compare[1] == 1500 && htim2.compare[0] == 1500);
  assert(htim1.compare[0] == 1500 && htim5.compare[0] == 1500);
  assert(htim3.compare[2] == 1500 && htim3.compare[3] == 1500);
  const unsigned initialWrites = bsp::outputAttempt;
  assert(driver.begin() && bsp::outputAttempt == initialWrites);

  const auto frame = nominal();
  auto applied = driver.applyFrameDetailed(frame);
  assert(applied.complete());
  checkEscPulse(applied, 54, -54);
  assert(frame.leftPropulsionPercent == 50 && frame.rightPropulsionPercent == 50);
  // Repeated frames always start from nominal 50, never the previous 54.
  for (unsigned cycle = 0; cycle < 100; ++cycle) {
    mockTick += 20;
    applied = driver.applyFrameDetailed(frame);
    checkEscPulse(applied, 54, -54);
  }
  applied = driver.applyFrameDetailed(nominal(-50, -50));
  checkEscPulse(applied, -54, 54);
  auto extreme = nominal(INT_MAX, INT_MIN);
  applied = driver.applyFrameDetailed(extreme);
  checkEscPulse(applied, 100, 100);  // clamp before right-side sign inversion

  for (uint16_t voltage : {uint16_t(0), uint16_t(6599), uint16_t(8401)}) {
    auto invalidVoltage = nominal();
    invalidVoltage.supplyMillivolts = voltage;
    applied = driver.applyFrameDetailed(invalidVoltage);
    assert(applied.complete());
    checkEscPulse(applied, 0, 0);
  }
  auto reference = nominal();
  reference.supplyMillivolts = 7400;
  checkEscPulse(driver.applyFrameDetailed(reference), 50, -50);
  reference.supplyMillivolts = 6600;
  checkEscPulse(driver.applyFrameDetailed(reference), 56, -56);
  reference.supplyMillivolts = 8400;
  checkEscPulse(driver.applyFrameDetailed(reference), 44, -44);

  // Servo command coordinates are preserved; do not mirror already-calibrated
  // left/right endpoints a second time when installing the L3 boundary.
  auto bounded = nominal();
  bounded.tailFrontDegrees = -1000;
  bounded.tailRearDegrees = 1000;
  bounded.pectoralLeftDegrees = -1000;
  bounded.pectoralRightDegrees = 1000;
  assert(driver.applyFrameDetailed(bounded).complete());
  assert(htim5.compare[1] == 722 && htim2.compare[0] == 2188);
  assert(htim1.compare[0] == 1000 && htim5.compare[0] == 2222);

  for (unsigned channel = 0; channel < 4; ++channel) {
    auto invalid = nominal();
    float *angles[] = {&invalid.tailFrontDegrees, &invalid.tailRearDegrees,
        &invalid.pectoralLeftDegrees, &invalid.pectoralRightDegrees};
    *angles[channel] = channel % 2 ? std::numeric_limits<float>::infinity()
                                 : std::numeric_limits<float>::quiet_NaN();
    const unsigned before = bsp::outputAttempt;
    assert(driver.applyFrameDetailed(invalid).writtenMask == 0);
    assert(bsp::outputAttempt == before); // invalid frame never partially writes
  }

  // A real latched ISR fault can occur after any prefix of the six writes.
  // Verify the actual Driver/BSP mask, not a fake driver's configured mask.
  for (unsigned successful = 0; successful < 6; ++successful) {
    bsp::outputAttempt = 0;
    bsp::latchAtAttempt = successful + 1;
    const auto partial = driver.applyFrameDetailed(frame);
    assert(partial.writtenMask == (1U << successful) - 1U);
    assert(!partial.complete());
  }
  bsp::latchAtAttempt = 0;

  // On a failed right ESC write the caller commits the original nominal left
  // request only, and keeps the prior nominal right command.
  ActuatorFrame committed = nominal(7, 9);
  const auto attempt = nominal(50, 60);
  bsp::outputAttempt = 0;
  bsp::latchAtAttempt = 6;
  applied = driver.applyFrameDetailed(attempt);
  assert(applied.writtenMask == 31 && applied.leftPropulsionPercent == 54);
  mergeWrittenFrame(committed, attempt, applied.writtenMask);
  assert(committed.leftPropulsionPercent == 50);
  assert(committed.rightPropulsionPercent == 9);
  bsp::latchAtAttempt = 0;

  // Each ESC timeout is measured from its own last successful submission.
  mockTick = 100;
  assert(driver.applyFrameDetailed(nominal()).complete());
  mockTick = 100 + RF_PROPULSION_TIMEOUT_MS - 1;
  assert(driver.update().writtenMask == 0);
  mockTick = 100 + RF_PROPULSION_TIMEOUT_MS;
  applied = driver.update();
  assert(applied.writtenMask == (PropulsionLeft | PropulsionRight));
  checkEscPulse(applied, 0, 0);
  assert(driver.update().writtenMask == 0);

  // A failed timeout neutral write is retried by the existing maintenance path;
  // successfully stopped channels are not spuriously reported again.
  mockTick = 5000;
  assert(driver.applyFrameDetailed(nominal()).complete());
  mockTick += RF_PROPULSION_TIMEOUT_MS;
  bsp::outputAttempt = 0;
  bsp::latchAtAttempt = 2;
  assert(driver.update().writtenMask == PropulsionLeft);
  bsp::latchAtAttempt = 0;
  assert(driver.update().writtenMask == PropulsionRight);
  assert(driver.update().writtenMask == 0);

  // Refreshing the left ESC must not renew the failed right ESC's old lease.
  mockTick = 10000;
  assert(driver.applyFrameDetailed(nominal()).complete());
  mockTick = 11000;
  bsp::outputAttempt = 0;
  bsp::latchAtAttempt = 6;
  assert(driver.applyFrameDetailed(nominal(60, 60)).writtenMask == 31);
  bsp::latchAtAttempt = 0;
  mockTick = 12000;
  assert(driver.update().writtenMask == PropulsionRight);
  mockTick = 13000;
  assert(driver.update().writtenMask == PropulsionLeft);

  std::puts("PASS: real L3 Device/PWM, polarity, single compensation, bounds, per-channel faults and existing timeout");
}
