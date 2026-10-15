#include "MotionController.h"

#include <algorithm>
#include <cmath>

#include "config/RuntimeConfig.h"
#include "bsp/Time.h"
#include "bsp/RuntimeSafety.h"
#include "config/MotionAllocationConfig.h"
#include "control/l0/IntentAdapter.h"
#include "control/l1/Command.h"
#include "control/l2/Allocator.h"

namespace {

constexpr uint32_t kControlPeriodMs = robofish::config::control::kPeriodMs;

uint8_t clampPercent(uint8_t value) {
  return std::min<uint8_t>(value, 100);
}

}  // namespace

uint32_t MotionController::nowMs() {
  return robofish::bsp::nowMs();
}

// 生命周期与共享内部操作 ----------------------------------------------------

MotionController::MotionController(RoboFishDriver &driver) : driver_(driver) {}

bool MotionController::begin() {
  if (started_.load()) return true;
  if (!driver_.ready()) return false;

  stateMutex_ = xSemaphoreCreateMutexStatic(&stateMutexStorage_);
  batteryQueue_ = xQueueCreateStatic(1, sizeof(robofish::BatterySample),
                                     batteryQueueBytes_, &batteryQueueStorage_);
  if (!stateMutex_ || !batteryQueue_) {
    return false;
  }

  controlTaskHandle_ = xTaskCreateStatic(
      controlTaskEntry, "MotionControl",
      sizeof(controlTaskStack_) / sizeof(StackType_t), this,
      RF_MOTION_CONTROL_TASK_PRIORITY,
      controlTaskStack_, &controlTaskStorage_);
  if (!controlTaskHandle_) {
    return false;
  }

  visionTaskHandle_ = xTaskCreateStatic(
      visionTaskEntry, "VisionControl", kVisionTaskStackBytes / sizeof(StackType_t),
      this, RF_VISION_LEASE_TASK_PRIORITY, visionTaskStack_,
      &visionTaskStorage_);
  if (!visionTaskHandle_) {
    return false;
  }

  started_.store(true);
  return true;
}

bool MotionController::takeState(TickType_t timeout) const {
  return stateMutex_ && xSemaphoreTake(stateMutex_, timeout) == pdTRUE;
}

void MotionController::giveState() const {
  xSemaphoreGive(stateMutex_);
}

bool MotionController::commandAllowed() const {
return started_.load() && driver_.ready() && !robofish::bsp::faultLatched();
}

void MotionController::onOperatorCommandLocked() {
  ++state_.safety.generation;
  state_.bodyActive = false;
  state_.remote.active = false;
  state_.vision.active = state_.vision.pidEnabled = state_.vision.pidPending = false;
}

bool MotionController::batteryMotionAllowedLocked() const {
  return robofish::batteryCondition(state_.diagnostics.battery, nowMs(),
      robofish::config::battery::kMaximumSampleAgeMs) == robofish::BatteryCondition::Normal;
}

bool MotionController::automationAllowedLocked(uint32_t generation) {
  return generation == state_.safety.generation && batteryMotionAllowedLocked() && state_.safety.mode == robofish::SafetyMode::Running &&
         state_.safety.owner == robofish::ControlOwner::Local &&
         !state_.remote.active && !outputRecovery_;
}

void MotionController::stopMotionLocked() {
  state_.drive.action = MotionAction::Stopped;
  resetVisionBrakeLocked();
}

bool MotionController::spiOnline(uint32_t now) const {
  return spiSeen_.load(std::memory_order_acquire) &&
      uint32_t(now - lastSpiReceiveMs_.load(std::memory_order_relaxed)) <
      robofish::config::safety::kSpiFreshnessMs;
}
void MotionController::publishSpiLink(uint32_t lastReceiveMs, bool seen) {
  if (!seen) return;
  lastSpiReceiveMs_.store(lastReceiveMs, std::memory_order_relaxed);
  spiSeen_.store(true, std::memory_order_release);
}
void MotionController::noteRadioCycle(uint32_t now) { radioHealth_.complete(now); }
void MotionController::serviceHealth(uint32_t now, bool complete) {
  if (complete) controlHealth_.complete(now);
  const bool healthy = radioHealth_.healthy(now, robofish::config::safety::kRadioDeadlineMs) &&
      controlHealth_.healthy(now, robofish::config::safety::kControlDeadlineMs) &&
      (!visionMonitored_.load(std::memory_order_acquire) ||
       visionHealth_.healthy(now, robofish::config::safety::kVisionDeadlineMs));
  robofish::bsp::serviceWatchdog(healthy, neutralSelfTest_.load(std::memory_order_acquire));
}
bool MotionController::motionAllowedLocked(robofish::ControlOwner owner) {
  if (owner != robofish::ControlOwner::LoRa &&
      state_.safety.mode == robofish::SafetyMode::Running &&
      state_.safety.owner == robofish::ControlOwner::LoRa &&
      uint32_t(nowMs() - state_.safety.lastTargetMs) < robofish::config::safety::kLoRaTargetLeaseMs)
    return false;
  const bool changingAuthority = state_.safety.mode != robofish::SafetyMode::Running || state_.safety.owner != owner;
  const bool accepted = state_.safety.motion(nowMs(), owner, batteryMotionAllowedLocked(), spiOnline(nowMs()),
      outputRecovery_ || robofish::bsp::faultLatched() || !robofish::bsp::bootConfirmed());
  if (accepted) {
    if (changingAuthority) clearTargetsLocked();
    pendingSurface_.store(false);
  }
  return accepted;
}
void MotionController::clearTargetsLocked() {
  const auto remoteMode = state_.remote.target.mode;
  stopMotionLocked(); state_.remote = {};
  state_.remote.target.mode = remoteMode; // Configuration survives; motion goals do not.
  state_.bodyActive = false;
  state_.bodyTarget = {};
  state_.auxiliary.pitchAngle = RF_PITCH_CENTER;
  state_.vision.active = state_.vision.pidEnabled = state_.vision.pidPending = false;
  robofish::algorithm::resetPid(state_.vision.xPid);
  robofish::algorithm::resetPid(state_.vision.yPid);
}
void MotionController::exitControlLocked(robofish::SafetyMode next) {
  clearTargetsLocked(); state_.safety.exit(next);
  pendingSurface_.store(false);
  if (next != robofish::SafetyMode::Surface) fullStopEpoch_.fetch_add(1);
}

// 命令接收与优先级仲裁 ----------------------------------------------------

bool MotionController::setMode(DriveMode mode) {
  if (!commandAllowed()) return false;
  if (mode != DriveMode::Bionic && mode != DriveMode::Prop &&
      mode != DriveMode::Hybrid) {
    return false;
  }
  if (!takeState(kCommandLockTimeout)) return false;
  onOperatorCommandLocked();
  state_.drive.mode = mode;
  stopMotionLocked();
  giveState();

  return true;
}

bool MotionController::cycleMode(DriveMode *newMode) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  onOperatorCommandLocked();
  switch (state_.drive.mode) {
    case DriveMode::Idle:
      state_.drive.mode = DriveMode::Bionic;
      break;
    case DriveMode::Bionic:
      state_.drive.mode = DriveMode::Prop;
      break;
    case DriveMode::Prop:
      state_.drive.mode = DriveMode::Hybrid;
      break;
    default:
      state_.drive.mode = DriveMode::Bionic;
      break;
  }
  stopMotionLocked();
  const DriveMode selected = state_.drive.mode;
  if (newMode) *newMode = selected;
  giveState();

  return true;
}

bool MotionController::submitManual(const MotionCommand &command) {
if (!commandAllowed()) return false;
  if (command.action != MotionAction::Stopped && pendingManualStop_.load()) return false;
  if (command.action == MotionAction::Stopped) {
    pendingManualStop_.store(true); fullStopEpoch_.fetch_add(1);
  }
  if (!takeState(kCommandLockTimeout)) return false;
  if (command.action != MotionAction::Stopped &&
      (state_.drive.mode == DriveMode::Idle || !motionAllowedLocked(robofish::ControlOwner::Local))) {
    giveState(); return false;
  }
  pendingManualStop_.store(false);
  if (command.action == MotionAction::Stopped) exitControlLocked();
  else {
    onOperatorCommandLocked(); resetVisionBrakeLocked();
    state_.drive.action = command.action;
    state_.drive.powerPercent = clampPercent(command.powerPercent);
    state_.drive.turnStrengthPercent = clampPercent(command.strengthPercent);
  }
  giveState(); return true;
}

bool MotionController::submitLoRa(const robofish::lora::Command &command) {
if (!commandAllowed() || command.code < 1 || command.code > 19 ||
      command.code == 8 || command.code == 18 || command.code == 19) return false;
  if (command.code == 17) { fullStopEpoch_.fetch_add(1); pendingFullStop_.store(true); }
  else if (command.code == 1) pendingHorizontalStop_.store(true);
  else if (pendingFullStop_.load() || pendingHorizontalStop_.load() || pendingManualStop_.load()) return false;
  if (!takeState(kCommandLockTimeout)) return false;
  if (command.code != 1 && command.code != 17 && !batteryMotionAllowedLocked()) {
    giveState(); return false;
  }
  if (robofish::isLoRaMotion(command.code) && !motionAllowedLocked(robofish::ControlOwner::LoRa)) {
    giveState(); return false;
  }
  if (command.code == 17) exitControlLocked();
  else if (command.code == 1) state_.safety.renewLoRa(nowMs());
  onOperatorCommandLocked();
  const bool accepted = applyRemoteCommand(state_.remote, command);
  const auto &target = state_.remote.target;
  state_.drive.mode = target.mode;
  state_.drive.action = target.surgePercent < 0 ? MotionAction::Backward
      : target.yawPercent < 0 ? MotionAction::Left
      : target.yawPercent > 0 ? MotionAction::Right
      : target.surgePercent > 0 ? MotionAction::Forward : MotionAction::Stopped;
  state_.vision.httpPriorityUntilMs = nowMs() + RF_VISION_HTTP_PRIORITY_HOLD_MS;
  if (command.code == 17) pendingFullStop_.store(false);
  if (command.code == 1) pendingHorizontalStop_.store(false);
  giveState(); return accepted;
}

bool MotionController::setPitch(int angle) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (!batteryMotionAllowedLocked()) { giveState(); return false; }
  onOperatorCommandLocked();
  state_.auxiliary.pitchAngle =
      std::clamp(angle, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE);
  giveState();
  return true;
}

bool MotionController::configureDirection(bool escReversed,
                                          bool steeringReversed,
                                          bool tailReversed) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  state_.remote.active = false;
  state_.direction.escReversed = escReversed;
  state_.direction.steeringReversed = steeringReversed;
  state_.direction.tailReversed = tailReversed;
  state_.bodyActive = false;
  state_.vision.active = false;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  stopMotionLocked();
  giveState();
  return true;
}

bool MotionController::setTailReversed(bool reversed) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  onOperatorCommandLocked();
  state_.direction.tailReversed = reversed;
  stopMotionLocked();
  giveState();
  return true;
}

bool MotionController::tailReversed() const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool reversed = state_.direction.tailReversed;
  giveState();
  return reversed;
}

bool MotionController::applyAutomationCommand(DriveMode mode, uint32_t generation) {
  if (!commandAllowed() ||
      (mode != DriveMode::Bionic && mode != DriveMode::Prop &&
       mode != DriveMode::Hybrid) ||
      !takeState(kCommandLockTimeout)) {
    return false;
  }
  if (!automationAllowedLocked(generation)) {
    giveState();
    return false;
  }
  state_.vision.active = false;
  state_.bodyActive = false;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  state_.drive.mode = mode;
  stopMotionLocked();
  giveState();
  return true;
}

bool MotionController::submitAutomation(const MotionCommand &command, uint32_t generation) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (!automationAllowedLocked(generation)) {
    giveState();
    return false;
  }
  if (state_.drive.mode == DriveMode::Idle &&
      command.action != MotionAction::Stopped) {
    giveState();
    return false;
  }
  state_.vision.active = false;
  state_.bodyActive = false;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  state_.drive.action = command.action;
  state_.drive.powerPercent = clampPercent(command.powerPercent);
  state_.drive.turnStrengthPercent = clampPercent(command.strengthPercent);
  if (command.action == MotionAction::Stopped) stopMotionLocked();
  giveState();
  return true;
}

bool MotionController::applyAutomationPitch(int angle, uint32_t generation) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (!automationAllowedLocked(generation)) {
    giveState();
    return false;
  }
  state_.vision.active = false;
  state_.bodyActive = false;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  state_.auxiliary.pitchAngle =
      std::clamp(angle, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE);
  giveState();
  return true;
}

bool MotionController::automationSafeStop(uint32_t generation) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  // 已接管的人工目标不能被旧脚本结束回调覆盖。
  if (!automationAllowedLocked(generation)) {
    giveState();
    return false;
  }
  state_.vision.active = false;
  state_.vision.pidEnabled = false;
  state_.vision.pidPending = false;
  exitControlLocked();
  giveState();
  return true;
}

// 显式安全停止入口；F405不监控Wi-Fi。
bool MotionController::triggerFailsafeLocked() {
if (state_.safety.mode != robofish::SafetyMode::Running) return false;
  pendingSurface_.store(true);
  return true;
}

bool MotionController::triggerFailsafe(const char *) {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool triggered = triggerFailsafeLocked();
  giveState();
  return triggered;
}

// FreeRTOS 周期任务与执行器提交 ------------------------------------------

// 两个任务只周期唤醒，不在循环中忙等。
void MotionController::controlTaskEntry(void *context) {
  static_cast<MotionController *>(context)->controlTask();
}

void MotionController::controlTask() {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    const bool completed = controlTick(nowMs());
    serviceHealth(nowMs(), completed);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
  }
}

// 运动输出任务是执行器的唯一周期提交位置。
bool MotionController::controlTick(uint32_t now) {
  if (robofish::bsp::faultLatched()) return robofish::bsp::controlledExitActive();
  const auto maintenance = driver_.update(); // 运行期维护也归唯一输出任务。
  robofish::mergeWrittenFrame(committed_, {}, maintenance.writtenMask);
  if (!takeState(kTickLockTimeout)) return false;
  robofish::BatterySample battery;
  if (xQueueReceive(batteryQueue_, &battery, 0) == pdTRUE)
    state_.diagnostics.battery = battery;
  if (maintenance.writtenMask & robofish::PropulsionLeft)
    state_.diagnostics.lastWrittenFrame.leftPropulsionPercent = 0;
  if (maintenance.writtenMask & robofish::PropulsionRight)
    state_.diagnostics.lastWrittenFrame.rightPropulsionPercent = 0;
  // If the first recovery exit could not acquire state, finish it now before
  // any successful neutral write is allowed to clear outputRecovery_.
  if (outputRecovery_ && (state_.safety.mode == robofish::SafetyMode::Running ||
                          state_.safety.mode == robofish::SafetyMode::Surface))
    exitControlLocked();
  const auto previousMode = state_.safety.mode;
  const bool fullStop = pendingFullStop_.exchange(false);
  const bool manualStop = pendingManualStop_.exchange(false);
  if (fullStop || manualStop) {
    pendingHorizontalStop_.store(false); exitControlLocked();
  } else if (pendingHorizontalStop_.exchange(false)) {
    onOperatorCommandLocked();
    applyRemoteCommand(state_.remote, robofish::lora::Command{0,1,false,0});
    state_.drive.action = MotionAction::Stopped;
  }
  if (pendingSurface_.exchange(false) && state_.safety.mode == robofish::SafetyMode::Running)
    exitControlLocked(robofish::SafetyMode::Surface);
  const bool batteryHealthy = batteryMotionAllowedLocked();
  if (state_.safety.update(now, batteryHealthy, robofish::config::safety::kLoRaTargetLeaseMs)) {
    clearTargetsLocked();
    if (state_.safety.mode != robofish::SafetyMode::Surface) fullStopEpoch_.fetch_add(1);
  }
  if (state_.safety.mode == robofish::SafetyMode::Surface && previousMode != state_.safety.mode) {
    surfaceFrame_ = committed_;
    surfaceFrame_.leftPropulsionPercent = surfaceFrame_.rightPropulsionPercent = 0;
    surfaceFrame_.flowAssisted = false; surfaceFrame_.supplyMillivolts = 0;
    allocationState_ = {};
  }
  const auto safetyMode = state_.safety.mode;
  const bool forcedNeutral = outputRecovery_ || safetyMode == robofish::SafetyMode::WaitCommand ||
      safetyMode == robofish::SafetyMode::BatteryStop;
  visionMonitored_.store(state_.vision.active, std::memory_order_release);
  const uint32_t dt = state_.actuator.lastControlMs == 0 ? kControlPeriodMs : now - state_.actuator.lastControlMs;
  state_.actuator.lastControlMs = now;
  auto &diagnostics = state_.diagnostics;
  diagnostics.lastPeriodMs = dt;
  diagnostics.maximumPeriodMs = std::max(diagnostics.maximumPeriodMs, dt);
  if (dt >= 2*kControlPeriodMs) diagnostics.skippedNominalPeriods += dt/kControlPeriodMs-1;
  robofish::control::l0::AdaptedIntent intent;
  if (state_.bodyActive) {
    intent.body = state_.bodyTarget;
    intent.mode = state_.drive.mode;
    intent.tailYawPercent = intent.body.yawPercent;
  } else if (state_.remote.active) {
    auto target = state_.remote.target;
    target.tailReversed = state_.direction.tailReversed;
    target.propulsionReversed = state_.direction.escReversed;
    target.steeringReversed = state_.direction.steeringReversed;
    intent = robofish::control::l0::fromRemote(target, state_.remote.immediateNeutral);
    state_.remote.immediateNeutral = false;
  } else {
    const bool moving = state_.drive.action != MotionAction::Stopped;
    const bool vision = state_.vision.active && state_.drive.mode == DriveMode::Hybrid;
    const uint8_t tailPower = vision ? state_.vision.tailPowerPercent : state_.drive.powerPercent;
    const uint8_t propPower = vision ? state_.vision.propulsionPowerPercent : state_.drive.powerPercent;
    const bool reverse = vision ? state_.vision.propulsionReversed : state_.drive.action == MotionAction::Backward;
    const bool turning = (state_.drive.action == MotionAction::Left || state_.drive.action == MotionAction::Right) && !(vision && reverse);
    const robofish::control::l0::ManualIntent manual{state_.drive.mode, state_.drive.action, tailPower, propPower,
        state_.drive.turnStrengthPercent, vision ? tailPower>0 : moving, vision ? propPower>0 : moving,
        reverse, turning, state_.direction.escReversed, state_.direction.steeringReversed, state_.direction.tailReversed, now};
    intent = robofish::control::l0::fromManual(manual, state_.auxiliary.pitchAngle,
        safetyMode == robofish::SafetyMode::Surface);
  }
  const MotionSource source = selectedSourceLocked();
  const bool invalidCommand = robofish::control::l1::validateCommand(intent.body) !=
      robofish::control::l1::CommandValidation::Valid;
  const bool unsupportedReverse = intent.body.enabled && intent.mode == DriveMode::Bionic &&
      intent.tailEnabled && (intent.independentDrives ?
          intent.tailDrivePercent < 0 : intent.body.surgePercent < 0);
  // Consume an unsupported request and invalidate asynchronous producers. An old
  // forward goal must not return when the user later changes the drive mode.
  if (invalidCommand || unsupportedReverse) exitControlLocked();
  const auto &voltage = state_.diagnostics.battery;
  const uint16_t supplyMillivolts = robofish::usableBatteryMillivolts(
      voltage, now, robofish::config::battery::kMaximumSampleAgeMs);
  giveState();
  // 相位/渐变只属于此任务；耗时计算和设备提交全部在共享锁之外。
  robofish::control::l2::Request request;
  request.mode = intent.mode;
  request.surgePercent = intent.body.surgePercent;
  request.yawPercent = intent.body.yawPercent;
  request.pitchPercent = intent.body.pitchPercent;
  request.rollPercent = intent.body.rollPercent;
  request.enabled = intent.body.enabled && !invalidCommand;
  request.outputScale = intent.outputScale;
  request.allowFlowAssist = intent.allowFlowAssist;
  request.immediate = intent.immediate;
  request.independentDrives = intent.independentDrives;
  request.tailDrivePercent = intent.tailDrivePercent;
  request.propulsionDrivePercent = intent.propulsionDrivePercent;
  request.independentTailYaw = intent.independentTailYaw;
  request.tailYawPercent = intent.tailYawPercent;
  request.tailEnabled = intent.tailEnabled;
  request.propulsionEnabled = intent.propulsionEnabled;
  robofish::control::l2::Result allocation;
  if (!forcedNeutral && safetyMode == robofish::SafetyMode::Running)
    allocation = robofish::control::l2::allocate(request,
        robofish::config::motion::makeRecipe(request.mode),
        robofish::config::motion::makeGeometry(), committed_, allocationState_, now, dt);
  else if (safetyMode == robofish::SafetyMode::Surface) allocation.candidateFrame = surfaceFrame_;
  if (invalidCommand) allocation.limitedReasons |= robofish::control::l2::InvalidInput;
  if (allocation.limitedReasons & robofish::control::l2::InvalidInput) {
    pendingFullStop_.store(true);
    fullStopEpoch_.fetch_add(1);
  }
  auto output = allocation.candidateFrame;
  // History is nominal. Installation polarity and voltage compensation occur
  // exactly once in L3; only successfully submitted channels advance history.
  output.supplyMillivolts = supplyMillivolts;
  const uint32_t requiredStopEpoch = fullStopEpoch_.load();
  const bool stopPending = pendingFullStop_.load() || requiredStopEpoch != appliedFullStopEpoch_;
  if (stopPending || forcedNeutral) {
    output = {};
    allocationState_ = {};
  }
  if (robofish::bsp::faultLatched()) return false;
  const auto applied = driver_.applyFrameDetailed(output);
  robofish::mergeWrittenFrame(committed_, output, applied.writtenMask);
  if (stopPending && applied.complete()) {
    appliedFullStopEpoch_ = requiredStopEpoch;
  }
  recordFrameAttempt(output, applied, now, source, &allocation);
  neutralSelfTest_.store(applied.complete() && forcedNeutral, std::memory_order_release);
  if (applied.complete()) {
    consecutiveWriteFailures_ = 0;
    outputRecovery_ = false; // The exit cleared the target; recovery never resumes it.
  } else {
    ++consecutiveWriteFailures_;
    if (consecutiveWriteFailures_ >= robofish::config::safety::kOutputNeutralFailures &&
        !outputRecovery_) {
      outputRecovery_ = true;
      if (takeState(kTickLockTimeout)) { exitControlLocked(); giveState(); }
      else fullStopEpoch_.fetch_add(1);
    }
    if (consecutiveWriteFailures_ >= robofish::config::safety::kOutputResetFailures) {
      robofish::bsp::controlledReset(robofish::bsp::FaultReason::OutputWriteFailure, consecutiveWriteFailures_);
      return true;
    }
  }
  return true; // A handled output failure is completed work, so it can feed IWDG.
}

// 状态查询与显示文本 ------------------------------------------------------

MotionSource MotionController::selectedSourceLocked() const {
  if ((state_.safety.mode == robofish::SafetyMode::Surface)) return MotionSource::Failsafe;
  if (state_.remote.active) return MotionSource::LoRa;
  if (state_.vision.active) return MotionSource::Vision;
  if (state_.bodyActive) return MotionSource::Algorithm;
  return MotionSource::LegacySdk;
}

void MotionController::recordFrameAttempt(const robofish::ActuatorFrame &frame,
    robofish::FrameApplyResult applied, uint32_t now, MotionSource source,
    const robofish::control::l2::Result *allocation) {
  if (!takeState(kTickLockTimeout)) return;
  auto &snapshot = state_.diagnostics;
  snapshot.lastAttemptFrame = frame;
  snapshot.lastAttemptSource = source;
  snapshot.frameSubmitted = applied.complete();
  snapshot.writtenMask = applied.writtenMask;
  const int previousLeft = snapshot.lastWrittenFrame.leftPropulsionPercent;
  const int previousRight = snapshot.lastWrittenFrame.rightPropulsionPercent;
  snapshot.lastWrittenFrame = committed_;
  snapshot.lastWrittenFrame.leftPropulsionPercent =
      applied.writtenMask & robofish::PropulsionLeft ? applied.leftPropulsionPercent : previousLeft;
  snapshot.lastWrittenFrame.rightPropulsionPercent =
      applied.writtenMask & robofish::PropulsionRight ? applied.rightPropulsionPercent : previousRight;
  if (allocation) {
    // Retained wire fields are display projections; allocation never reads them.
    state_.actuator.throttleOutput = RF_ESC_THROTTLE_NEUTRAL +
        (committed_.leftPropulsionPercent + committed_.rightPropulsionPercent) * 45 / 200;
    state_.actuator.steeringOutput = RF_ESC_STEERING_CENTER +
        (committed_.leftPropulsionPercent - committed_.rightPropulsionPercent) * 45 / 200;
    state_.actuator.leftPropulsionPercent = committed_.leftPropulsionPercent;
    state_.actuator.rightPropulsionPercent = committed_.rightPropulsionPercent;
    state_.actuator.tail1Output = committed_.tailFrontDegrees;
    state_.actuator.tail2Output = committed_.tailRearDegrees;
    snapshot.commonPropulsionPercent = allocation->commonPropulsionPercent;
    snapshot.differentialPropulsionPercent = allocation->differentialPropulsionPercent;
    snapshot.phaseRad = allocation->phaseRad;
    snapshot.limitedReasons = allocation->limitedReasons;
  }
  snapshot.lastAttemptMs = now;
  ++snapshot.frameAttempts;
  if (!applied.complete()) ++snapshot.frameFailures;
  giveState();
}

bool MotionController::publishBatterySample(const robofish::BatterySample &sample) {
  return started_.load() && batteryQueue_ &&
      xQueueOverwrite(batteryQueue_, &sample) == pdTRUE;
}

bool MotionController::observeLoRa(const robofish::lora::LinkStatus &snapshot) {
  if (!started_.load() || !takeState(kTickLockTimeout)) return false;
  state_.diagnostics.radio = snapshot;
  state_.diagnostics.radioObservedMs = nowMs();
  giveState();
  return true;
}

bool MotionController::getDiagnostics(MotionDiagnostics &snapshot) const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  snapshot = state_.diagnostics;
  snapshot.safetyMode = state_.safety.mode;
  snapshot.controlGeneration = state_.safety.generation;
  snapshot.consecutiveWriteFailures = consecutiveWriteFailures_;
  snapshot.lastFaultReason = robofish::bsp::lastFaultReason();
  snapshot.resetFlags = robofish::bsp::resetFlags();
  snapshot.startBlockReasons = 0;
  if (!batteryMotionAllowedLocked()) snapshot.startBlockReasons |= robofish::StartBattery;
  if (!spiOnline(nowMs())) snapshot.startBlockReasons |= robofish::StartSpiOffline;
  if (state_.safety.mode != robofish::SafetyMode::Running) snapshot.startBlockReasons |= robofish::StartNewCommand;
  if (robofish::bsp::faultLatched()) snapshot.startBlockReasons |= robofish::StartFault;
  if (outputRecovery_) snapshot.startBlockReasons |= robofish::StartOutputRecovery;
  if (!robofish::bsp::bootConfirmed()) snapshot.startBlockReasons |= robofish::StartBootTrial;
  snapshot.selectedSource = selectedSourceLocked();
  snapshot.batteryAgeMs = nowMs() - snapshot.battery.sampledAtMs;
  snapshot.remoteActive = state_.remote.active;
  snapshot.remoteTargetEnabled = state_.remote.active && state_.remote.target.enabled;
  if (snapshot.remoteActive) {
    snapshot.surgePercent = state_.remote.target.surgePercent;
    snapshot.yawPercent = state_.remote.target.yawPercent;
    snapshot.heavePercent = state_.remote.target.heavePercent;
    snapshot.knob = state_.remote.target.knob;
  }
  if (snapshot.attitudeAvailable) snapshot.attitudeAgeMs = nowMs() - snapshot.attitude.sampledAtMs;
  giveState();
  return true;
}

bool MotionController::getStatus(MotionStatus &status) const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  status = {state_.drive.mode,
            state_.drive.action,
            state_.drive.powerPercent,
            state_.drive.turnStrengthPercent,
            state_.auxiliary.pitchAngle,
            (state_.safety.mode == robofish::SafetyMode::Surface),
            static_cast<int>(std::lround(state_.actuator.tail1Output)),
            static_cast<int>(std::lround(state_.actuator.tail2Output)),
            state_.actuator.throttleOutput,
            state_.actuator.steeringOutput,
            state_.vision.appliedSequence};
  giveState();
  return true;
}

bool MotionController::failsafeActive() const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool active = (state_.safety.mode == robofish::SafetyMode::Surface);
  giveState();
  return active;
}

const char *MotionController::modeName(DriveMode mode) {
  switch (mode) {
    case DriveMode::Idle:
      return "IDLE";
    case DriveMode::Bionic:
      return "BIONIC";
    case DriveMode::Prop:
      return "PROP";
    case DriveMode::Hybrid:
      return "HYBRID";
    default:
      return "UNKNOWN";
  }
}

void MotionController::update() {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return;
  if ((state_.safety.mode == robofish::SafetyMode::Surface)) state_.auxiliary.pitchAngle = RF_PITCH_MIN_ANGLE;
  giveState();
}
