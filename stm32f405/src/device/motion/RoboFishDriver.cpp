#include "RoboFishDriver.h"
#include <algorithm>
#include <cmath>
#include "device/motion/EscCalibration.h"
#include "config/ControlConfig.h"
#include "bsp/ServoPwm.h"
#include "bsp/Time.h"

namespace {
constexpr TickType_t kDeviceCommandTimeout = pdMS_TO_TICKS(robofish::config::control::kDeviceMaintenanceTimeoutMs);
constexpr TickType_t kRuntimeTimeout = pdMS_TO_TICKS(robofish::config::control::kDeviceRuntimeLockTimeoutMs);
}

bool RoboFishDriver::begin() {
  if (ready_.load()) return true;

  bool attached = ServoPwm::configureTimers();
  attached &= tail_.begin();
  attached &= pectoral_.begin();
  attached &= propulsion_.begin();

  attached &= stopAllOutputs(kDeviceCommandTimeout);
  ready_.store(attached);

  return attached;
}

robofish::FrameApplyResult RoboFishDriver::applyFrameDetailed(const robofish::ActuatorFrame &frame) {
  if (!ready_.load() || !std::isfinite(frame.tailFrontDegrees) ||
      !std::isfinite(frame.tailRearDegrees) || !std::isfinite(frame.pectoralLeftDegrees) ||
      !std::isfinite(frame.pectoralRightDegrees)) return {};
  // 六路PWM更新非原子；调用方按通道记录成功提交。
  const auto angle = [](float value, int low, int high) {
    return static_cast<int>(std::clamp(value, static_cast<float>(low), static_cast<float>(high)));
  };
  const uint8_t tail = tail_.applyBothMask(angle(frame.tailFrontDegrees, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE),
      angle(frame.tailRearDegrees, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE), kRuntimeTimeout);
  namespace c = robofish::config::lora;
  const uint8_t chest = pectoral_.setBothMask(angle(frame.pectoralLeftDegrees, c::kPectoralLeftMinimum, c::kPectoralLeftMaximum),
      angle(frame.pectoralRightDegrees, c::kPectoralRightMinimum, c::kPectoralRightMaximum), kRuntimeTimeout);
  // L2 forward-positive commands become installed electrical commands once.
  const int leftPercent = robofish::device::propulsionElectricalPercent(
      frame.leftPropulsionPercent, robofish::device::PropulsionSide::Left,
      frame.supplyMillivolts);
  const int rightPercent = robofish::device::propulsionElectricalPercent(
      frame.rightPropulsionPercent, robofish::device::PropulsionSide::Right,
      frame.supplyMillivolts);
  const uint8_t prop = propulsion_.setDifferentialMask(leftPercent, rightPercent, kRuntimeTimeout);
  return {static_cast<uint8_t>(tail | (chest << 2) | (prop << 4)),
          static_cast<int16_t>(leftPercent), static_cast<int16_t>(rightPercent)};
}

bool RoboFishDriver::stopAllOutputs(TickType_t timeout) {
  const bool tail = tail_.applyBoth(robofish::config::lora::kTailFrontCenter,
                robofish::config::lora::kTailRearCenter, timeout);
  const bool chest = pectoral_.setBoth(robofish::config::lora::kPectoralLeftCenter,
      robofish::config::lora::kPectoralRightCenter, timeout);
  const bool prop = propulsion_.stop(timeout);
  return tail && chest && prop;
}

robofish::FrameApplyResult RoboFishDriver::update() {
  if (!ready_.load()) return {};
  const uint8_t mask = propulsion_.updateMask(robofish::bsp::nowMs(), 0);
  return {static_cast<uint8_t>(mask << 4)};
}

bool RoboFishDriver::ready() const { return ready_.load(); }
