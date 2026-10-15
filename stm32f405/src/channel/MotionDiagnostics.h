#pragma once
#include "algorithm/SafetyPolicy.h"
#include "channel/ActuatorFrame.h"
#include "channel/AttitudeSample.h"
#include "channel/BatterySample.h"
#include "channel/FrameApplyResult.h"
#include "channel/LoRaLinkStatus.h"

enum class MotionSource : uint8_t { LegacySdk = 0, LoRa = 1, Vision = 2, Algorithm = 3, Failsafe = 4 };
struct MotionDiagnostics {
  // 当前目标来源可已改变；lastAttempt来源/输出/结果在同一锁内形成一致快照。
  MotionSource selectedSource = MotionSource::LegacySdk;
  bool remoteActive = false;
  bool remoteTargetEnabled = false;
  float surgePercent = 0;
  float yawPercent = 0;
  float heavePercent = 0;
  uint16_t knob = 0;
  MotionSource lastAttemptSource = MotionSource::LegacySdk;
  robofish::ActuatorFrame lastAttemptFrame;
  robofish::ActuatorFrame lastWrittenFrame;
  uint8_t writtenMask = 0;
  float commonPropulsionPercent = 0;
  float differentialPropulsionPercent = 0;
  double phaseRad = 0;
  uint32_t limitedReasons = 0;
  robofish::lora::LinkStatus radio;
  uint32_t radioObservedMs = 0;
  bool frameSubmitted = false;  // API合计结果；多路PWM更新非原子，失败可含部分成功。
  uint32_t lastAttemptMs = 0;
  uint32_t frameAttempts = 0;
  uint32_t frameFailures = 0;
  uint32_t lastPeriodMs = 0;
  uint32_t maximumPeriodMs = 0;
  uint32_t skippedNominalPeriods = 0;
  bool attitudeAvailable = false; // Raw history exists; freshness is in getFeedback().
  uint32_t attitudeAgeMs = 0;
  AttitudeSample attitude;
  robofish::BatterySample battery;
  uint32_t batteryAgeMs = 0;
  robofish::SafetyMode safetyMode = robofish::SafetyMode::WaitCommand;
  uint32_t startBlockReasons = robofish::StartNewCommand;
  uint32_t controlGeneration = 0;
  uint32_t consecutiveWriteFailures = 0;
  uint32_t lastFaultReason = 0, resetFlags = 0;
};

inline const char *motionSourceName(MotionSource source) {
  switch (source) {
    case MotionSource::LoRa: return "lora";
    case MotionSource::Vision: return "vision";
    case MotionSource::Algorithm: return "algorithm";
    case MotionSource::Failsafe: return "failsafe";
    default: return "sdk";
  }
}
