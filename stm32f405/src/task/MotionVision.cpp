#include "task/MotionController.h"
#include <algorithm>
#include <cmath>
#include "algorithm/PidController.h"
#include "control/l0/VisionMotionPolicy.h"

namespace {
constexpr uint32_t kControlPeriodMs = robofish::config::control::kPeriodMs;
constexpr robofish::algorithm::PidConfig kSteeringPid = {
    RF_VISION_X_KP,
    RF_VISION_X_KI,
    RF_VISION_X_KD,
    -RF_VISION_PID_INTEGRAL_LIMIT,
    RF_VISION_PID_INTEGRAL_LIMIT,
    RF_VISION_PID_DEADBAND_M,
};

constexpr robofish::algorithm::PidConfig kPowerPid = {
    RF_VISION_Y_KP,
    RF_VISION_Y_KI,
    RF_VISION_Y_KD,
    0.0f,
    RF_VISION_PID_INTEGRAL_LIMIT,
    RF_VISION_PID_DEADBAND_M,
};

}  // namespace

void MotionController::resetVisionBrakeLocked() {
  state_.vision.tailPowerPercent = 0;
  state_.vision.propulsionPowerPercent = 0;
  state_.vision.speedMps = 0.0f;
  state_.vision.curveSeverity = 0.0f;
  state_.vision.brakeRequested = false;
  state_.vision.brakePhase = VisionBrakePhase::Cruise;
  state_.vision.brakePhaseStartedMs = 0;
  state_.vision.propulsionReversed = false;
}

// 视觉目标：HTTP 优先于 UDP，所有来源最终覆盖同一份 latest-value 状态。
VisionControlResult MotionController::applyVisionControl(
    const VisionSetpoint &setpoint) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) {
    return VisionControlResult::Rejected;
  }
  const VisionControlResult result = applyVisionCommandLocked(setpoint);
  giveState();
  return result;
}

VisionControlResult MotionController::applyVisionUdp(
    const VisionSetpoint &setpoint) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) {
    return VisionControlResult::Rejected;
  }
  const uint32_t now = nowMs();
  if ((state_.safety.mode == robofish::SafetyMode::Surface) || state_.remote.active ||
      static_cast<int32_t>(state_.vision.httpPriorityUntilMs - now) > 0) {
    giveState();
    return VisionControlResult::Rejected;
  }
  const VisionControlResult result = applyVisionCommandLocked(setpoint);
  giveState();
  return result;
}

void MotionController::noteHttpPriority() {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return;
  state_.vision.httpPriorityUntilMs =
      nowMs() + RF_VISION_HTTP_PRIORITY_HOLD_MS;
  state_.bodyActive = false;
  ++state_.safety.generation;
  // Invalidate any in-flight UDP lease immediately.  The HTTP handler will
  // install a new command after this call when appropriate.
  state_.vision.active = false;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  stopMotionLocked();
  giveState();
}

VisionControlResult MotionController::applyVisionCommandLocked(
    const VisionSetpoint &setpoint) {
  if (!batteryMotionAllowedLocked() || setpoint.powerPercent > 100 || setpoint.steeringPercent < -100 ||
      setpoint.steeringPercent > 100) {
    return VisionControlResult::Rejected;
  }
  if (state_.vision.active) {
    const uint32_t delta = setpoint.sequence - state_.vision.sequence;
    if (delta == 0 || delta >= 0x80000000u) {
      return VisionControlResult::StaleSequence;
    }
  }

  if (state_.safety.mode != robofish::SafetyMode::Running ||
      state_.safety.owner != robofish::ControlOwner::Local ||
      setpoint.controlGeneration != state_.safety.generation || outputRecovery_)
    return VisionControlResult::Rejected;
  state_.remote.active = false;
  state_.bodyActive = false;
  resetVisionBrakeLocked();
  state_.vision.active = true;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  state_.vision.sequence = setpoint.sequence;
  state_.vision.lastCommandMs = nowMs();
  state_.drive.mode = DriveMode::Hybrid;
  state_.drive.powerPercent = setpoint.powerPercent;
  state_.vision.tailPowerPercent = setpoint.powerPercent;
  state_.vision.propulsionPowerPercent = setpoint.powerPercent;
  state_.vision.speedMps = 0.0f;
  state_.vision.curveSeverity = 0.0f;
  state_.vision.brakeRequested = setpoint.brakeRequested;
  state_.drive.turnStrengthPercent =
      static_cast<uint8_t>(std::abs(setpoint.steeringPercent));

  if (setpoint.powerPercent == 0) {
    stopMotionLocked();
  } else if (setpoint.steeringPercent < 0) {
    state_.drive.action = MotionAction::Left;
  } else if (setpoint.steeringPercent > 0) {
    state_.drive.action = MotionAction::Right;
  } else {
    state_.drive.action = MotionAction::Forward;
  }
  return VisionControlResult::Accepted;
}

VisionControlResult MotionController::applyVisionPid(uint32_t sequence,
                                                     float xErrorM,
                                                     float yErrorM,
                                                     float speedMps,
                                                     float curveSeverity,
                                                     bool brakeRequested, uint32_t generation) {
  if (!commandAllowed() || !std::isfinite(xErrorM) ||
      !std::isfinite(yErrorM) || !std::isfinite(speedMps) ||
      !std::isfinite(curveSeverity) || std::abs(xErrorM) > robofish::config::control::kVisionMaximumErrorMeters ||
      yErrorM < 0.0f || yErrorM > robofish::config::control::kVisionMaximumErrorMeters || speedMps < 0.0f ||
      speedMps > robofish::config::control::kVisionMaximumSpeedMps || curveSeverity < 0.0f || curveSeverity > 1.0f ||
      !takeState(kCommandLockTimeout)) {
    return VisionControlResult::Rejected;
  }

  if (state_.vision.active) {
    const uint32_t delta = sequence - state_.vision.sequence;
    if (delta == 0 || delta >= 0x80000000u) {
      giveState();
      return VisionControlResult::StaleSequence;
    }
  }

  if (!batteryMotionAllowedLocked()) {
    giveState();
    return VisionControlResult::Rejected;
  }
  if (state_.safety.mode != robofish::SafetyMode::Running ||
      state_.safety.owner != robofish::ControlOwner::Local ||
      generation != state_.safety.generation || outputRecovery_) {
    giveState(); return VisionControlResult::Rejected;
  }
  const bool startingPid = !state_.vision.pidEnabled;
  state_.bodyActive = false;
  state_.remote.active = false;
  if (startingPid) resetVisionBrakeLocked();
  state_.vision.active = true;
  state_.vision.pidEnabled = true;
  state_.vision.pidPending = true;
  state_.vision.sequence = sequence;
  state_.vision.lastCommandMs = nowMs();
  state_.drive.mode = DriveMode::Hybrid;
  state_.vision.xErrorM = xErrorM;
  state_.vision.yErrorM = yErrorM;
  state_.vision.speedMps = speedMps;
  state_.vision.curveSeverity = curveSeverity;
  state_.vision.brakeRequested = brakeRequested;
  if (startingPid) {
    robofish::algorithm::resetPid(state_.vision.xPid);
    robofish::algorithm::resetPid(state_.vision.yPid);
    state_.vision.xPid.previousError = xErrorM;
    state_.vision.yPid.previousError = yErrorM;
    state_.vision.pidPreviousMs = state_.vision.lastCommandMs;
  }
  giveState();
  return VisionControlResult::Accepted;
}

void MotionController::visionTaskEntry(void *context) {
  static_cast<MotionController *>(context)->visionTask();
}

void MotionController::visionTask() {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (visionTick(nowMs())) visionHealth_.complete(nowMs());
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
  }
}

// 视觉任务计算并覆盖最新执行器目标，不直接写入硬件。
bool MotionController::visionTick(uint32_t now) {
  if (!takeState(kTickLockTimeout)) return false;
  if (state_.vision.active && state_.vision.pidEnabled &&
      state_.vision.pidPending) {
    const float dt = std::clamp(
        (now - state_.vision.pidPreviousMs) * 0.001f, robofish::config::control::kVisionMinimumDtSeconds, robofish::config::control::kVisionMaximumDtSeconds);
    state_.vision.pidPreviousMs = now;
    state_.vision.pidPending = false;

    const float steeringOutput = robofish::algorithm::updatePid(
        state_.vision.xErrorM, dt, kSteeringPid, state_.vision.xPid);
    const float powerOutput = robofish::algorithm::updatePid(
        state_.vision.yErrorM, dt, kPowerPid, state_.vision.yPid);
    // PC x_error is left-positive. L0 converts that legacy path convention;
    // direct VisionSetpoint.steeringPercent and the resulting L1 yaw are right-positive.
    const auto demand = robofish::control::l0::mapVisionDemand(
        steeringOutput, powerOutput, state_.vision.curveSeverity);

    state_.vision.tailPowerPercent = demand.tailPowerPercent;
    state_.vision.propulsionPowerPercent = demand.propulsionPowerPercent;
    state_.drive.powerPercent = demand.propulsionPowerPercent;
    state_.drive.turnStrengthPercent = demand.turnStrengthPercent;
    state_.drive.action = demand.action;
    state_.vision.appliedSequence = state_.vision.sequence;
  }

  if (state_.vision.active) {
    if (!state_.vision.brakeRequested) {
      state_.vision.brakePhase = VisionBrakePhase::Cruise;
      state_.vision.brakePhaseStartedMs = 0;
      state_.vision.propulsionReversed = false;
    } else {
      if (state_.vision.brakePhase == VisionBrakePhase::Cruise) {
        state_.vision.brakePhase = VisionBrakePhase::Neutralizing;
        state_.vision.brakePhaseStartedMs = now;
      }

      state_.vision.tailPowerPercent = 0;
      state_.vision.propulsionPowerPercent = 0;
      state_.drive.powerPercent = 0;
      state_.vision.propulsionReversed = false;

      switch (state_.vision.brakePhase) {
        case VisionBrakePhase::Neutralizing:
          if (std::abs(state_.actuator.leftPropulsionPercent) <=
                  robofish::config::control::kVisionBrakeNeutralPercent &&
              std::abs(state_.actuator.rightPropulsionPercent) <=
                  robofish::config::control::kVisionBrakeNeutralPercent) {
            state_.vision.brakePhase = VisionBrakePhase::NeutralDwell;
            state_.vision.brakePhaseStartedMs = now;
          }
          break;
        case VisionBrakePhase::NeutralDwell:
          if (now - state_.vision.brakePhaseStartedMs >=
              RF_VISION_BRAKE_NEUTRAL_DWELL_MS) {
            state_.vision.brakePhase = VisionBrakePhase::ReverseBrake;
            state_.vision.brakePhaseStartedMs = now;
          }
          break;
        case VisionBrakePhase::ReverseBrake: {
          const uint32_t reverseElapsed =
              now - state_.vision.brakePhaseStartedMs;
          const bool speedSettled =
              reverseElapsed >= RF_VISION_BRAKE_REVERSE_MIN_MS &&
              state_.vision.speedMps <= RF_VISION_BRAKE_SETTLED_SPEED_MPS;
          if (speedSettled ||
              reverseElapsed >= RF_VISION_BRAKE_REVERSE_MAX_MS) {
            state_.vision.brakePhase = VisionBrakePhase::Hold;
            state_.vision.brakePhaseStartedMs = now;
          } else {
            state_.vision.propulsionPowerPercent =
                RF_VISION_BRAKE_REVERSE_POWER_PERCENT;
            state_.drive.powerPercent = state_.vision.propulsionPowerPercent;
            state_.vision.propulsionReversed = true;
          }
          break;
        }
        case VisionBrakePhase::Hold:
          // Reverse braking may settle outside the final stop circle. Keep the
          // latest visual steering command and crawl forward at the same
          // current-limited power until the PC reports arrival and sends STOP.
          state_.vision.tailPowerPercent =
              RF_VISION_BRAKE_REAPPROACH_POWER_PERCENT;
          state_.vision.propulsionPowerPercent =
              RF_VISION_BRAKE_REAPPROACH_POWER_PERCENT;
          state_.drive.powerPercent = state_.vision.propulsionPowerPercent;
          state_.vision.propulsionReversed = false;
          break;
        case VisionBrakePhase::Cruise:
          break;
      }
    }
  }
  if (state_.vision.active &&
      now - state_.vision.lastCommandMs >= RF_VISION_COMMAND_TIMEOUT_MS) {
    state_.vision.active = false;
    state_.vision.pidEnabled = false;
    state_.vision.pidPending = false;
    robofish::algorithm::resetPid(state_.vision.xPid);
    robofish::algorithm::resetPid(state_.vision.yPid);
    exitControlLocked();
  }
  giveState();
  return true;
}
