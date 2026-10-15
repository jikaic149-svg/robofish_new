#pragma once

#include <atomic>
#include "algorithm/TaskHealth.h"
#include <stdint.h>

#include "channel/MotionTypes.h"
#include "control/l2/Allocator.h"
#include "channel/ControlFeedback.h"
#include "device/motion/RoboFishDriver.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <task.h>
#include "task/MotionControllerState.h"

// 运动命令的唯一仲裁者，也是执行器输出的唯一提交者。
class MotionController {
 public:
  explicit MotionController(RoboFishDriver &driver);

  // 生命周期
  bool begin();
  void update();

  // 新人工运动可授权；辅助配置不能解除等待/上浮，旧异步结果随代次失效。
  bool setMode(DriveMode mode);
  bool cycleMode(DriveMode *newMode = nullptr);
  bool submitManual(const MotionCommand &command);
  bool submitLoRa(const robofish::lora::Command &command);
  bool setPitch(int angle);
  bool setTailReversed(bool reversed);
  bool tailReversed() const;
  bool configureDirection(bool escReversed, bool steeringReversed,
                          bool tailReversed);

  // Automation requires an authorized local session and its current generation.
  bool applyAutomationCommand(DriveMode mode, uint32_t generation = 0);
  bool submitAutomation(const MotionCommand &command, uint32_t generation = 0);
  bool applyAutomationPitch(int angle, uint32_t generation = 0);
  bool automationSafeStop(uint32_t generation = 0);
  // Algorithms supply normalized body axes after obtaining a local generation.
  // This entry cannot authorize movement or bypass existing stop/battery gates.
  bool submitBodyCommand(DriveMode mode,
      const robofish::control::l1::BodyCommand &command, uint32_t generation);

  // 视觉控制：只保存最新目标，不建立历史命令队列。
  VisionControlResult applyVisionControl(const VisionSetpoint &setpoint);
  VisionControlResult applyVisionUdp(const VisionSetpoint &setpoint);
  VisionControlResult applyVisionPid(uint32_t sequence, float xErrorM,
                                     float yErrorM, float speedMps,
                                     float curveSeverity,
                                     bool brakeRequested, uint32_t generation = 0);
  void noteHttpPriority();

  // 网络和安全状态。
  bool triggerFailsafe(const char *reason);

  // 只读状态。
  bool getStatus(MotionStatus &status) const;
  bool getDiagnostics(MotionDiagnostics &snapshot) const;
  bool publishBatterySample(const robofish::BatterySample &sample);
  bool observeLoRa(const robofish::lora::LinkStatus &snapshot);
  void publishSpiLink(uint32_t lastReceiveMs, bool seen);
  void noteRadioCycle(uint32_t now);
  bool submitAttitude(const AttitudeSample &sample);
  bool clearAttitude();
  bool submitDepth(const robofish::DepthSample &sample);
  bool submitVelocity(const robofish::VelocitySample &sample);
  // Successful query can still contain no available observation. Algorithms
  // must check each flag; maximumAgeMs is the caller's observation-age budget.
  bool getFeedback(robofish::ControlFeedback &snapshot, uint32_t maximumAgeMs) const;
  // Explicit producer restart: clears all three histories, without moving fish.
  bool clearFeedback();
  bool failsafeActive() const;
  static const char *modeName(DriveMode mode);

 private:
  static constexpr uint32_t kControlTaskStackBytes = robofish::config::control::kControlTaskStackBytes;
  static constexpr uint32_t kVisionTaskStackBytes = robofish::config::control::kVisionTaskStackBytes;
  static constexpr TickType_t kCommandLockTimeout = pdMS_TO_TICKS(robofish::config::control::kCommandLockTimeoutMs);
  static constexpr TickType_t kTickLockTimeout = pdMS_TO_TICKS(robofish::config::control::kTickLockTimeoutMs);

  // FreeRTOS 周期任务。
  static void controlTaskEntry(void *context);
  void controlTask();
  static void visionTaskEntry(void *context);
  void visionTask();
  bool visionTick(uint32_t nowMs);
  bool controlTick(uint32_t nowMs);
  void serviceHealth(uint32_t now, bool controlCompleted);
  bool spiOnline(uint32_t now) const;
  bool motionAllowedLocked(robofish::ControlOwner owner);
  void clearTargetsLocked();
  void exitControlLocked(robofish::SafetyMode next = robofish::SafetyMode::WaitCommand);
  void recordFrameAttempt(const robofish::ActuatorFrame &frame, robofish::FrameApplyResult applied,
                          uint32_t nowMs, MotionSource source,
                          const robofish::control::l2::Result *allocation = nullptr);
  MotionSource selectedSourceLocked() const;

  // 所有 Locked 函数只允许在持有 stateMutex_ 时调用。
  static uint32_t nowMs();
  bool takeState(TickType_t timeout) const;
  void giveState() const;
  bool commandAllowed() const;
  void onOperatorCommandLocked();
  bool automationAllowedLocked(uint32_t generation);
  bool batteryMotionAllowedLocked() const;
  VisionControlResult applyVisionCommandLocked(
      const VisionSetpoint &setpoint);
  void stopMotionLocked();
  void resetVisionBrakeLocked();
  bool triggerFailsafeLocked();

  RoboFishDriver &driver_;
  mutable StaticSemaphore_t stateMutexStorage_{};
  mutable SemaphoreHandle_t stateMutex_ = nullptr;

  StaticTask_t controlTaskStorage_{};
  alignas(16) StackType_t controlTaskStack_[kControlTaskStackBytes /
                                             sizeof(StackType_t)]{};
  TaskHandle_t controlTaskHandle_ = nullptr;

  StaticTask_t visionTaskStorage_{};
  alignas(16) StackType_t visionTaskStack_[kVisionTaskStackBytes /
                                           sizeof(StackType_t)]{};
  TaskHandle_t visionTaskHandle_ = nullptr;

  std::atomic_bool started_{false};
  std::atomic_bool pendingFullStop_{false};
  std::atomic_uint32_t fullStopEpoch_{0};
  std::atomic_bool pendingHorizontalStop_{false};
  std::atomic_bool pendingManualStop_{false};
  robofish::control::l2::State allocationState_{}; // Only the output task advances phase.
  robofish::ActuatorFrame committed_{}; // Nominal successfully submitted channels.
  std::atomic_bool pendingSurface_{false};
  std::atomic<uint32_t> lastSpiReceiveMs_{0};
  std::atomic_bool spiSeen_{false};
  robofish::TaskHealth radioHealth_, controlHealth_, visionHealth_;
  std::atomic_bool visionMonitored_{false};
  std::atomic_bool neutralSelfTest_{false};
  robofish::ActuatorFrame surfaceFrame_{};
  std::atomic<uint32_t> consecutiveWriteFailures_{0};
  std::atomic_bool outputRecovery_{false};
  uint32_t appliedFullStopEpoch_ = 0; // 全停止至少成功提交一次，再运行最新后续目标。
  StaticQueue_t batteryQueueStorage_{};
  alignas(4) uint8_t batteryQueueBytes_[sizeof(robofish::BatterySample)]{};
  QueueHandle_t batteryQueue_ = nullptr;
  MotionControllerState state_{};
};
