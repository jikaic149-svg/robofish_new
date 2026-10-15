#pragma once

#include <stdint.h>
#include "algorithm/SafetyPolicy.h"

#include "algorithm/PidController.h"
#include "channel/MotionTypes.h"
#include "config/RuntimeConfig.h"
#include "task/RemoteControlState.h"
#include "channel/MotionDiagnostics.h"
#include "channel/ControlFeedback.h"

// 以下类型只描述 MotionController 内部状态，不参与跨模块数据传输。
enum class VisionBrakePhase : uint8_t {
  Cruise = 0,
  Neutralizing,
  NeutralDwell,
  ReverseBrake,
  Hold,
};

struct DriveState {
  DriveMode mode = DriveMode::Idle;
  MotionAction action = MotionAction::Stopped;
  uint8_t powerPercent = 100;
  uint8_t turnStrengthPercent = 100;
};

struct AuxiliaryState {
  int pitchAngle = RF_PITCH_CENTER;
};

using SafetyState = robofish::SafetyPolicy;

struct DirectionState {
  bool escReversed = false;
  bool steeringReversed = false;
  bool tailReversed = false;
};

struct VisionState {
  bool active = false;
  uint32_t sequence = 0;
  uint32_t appliedSequence = 0;
  uint32_t lastCommandMs = 0;
  uint32_t httpPriorityUntilMs = 0;

  bool pidEnabled = false;
  bool pidPending = false;
  float xErrorM = 0.0f;
  float yErrorM = 0.0f;
  robofish::algorithm::PidState xPid;
  robofish::algorithm::PidState yPid;
  uint32_t pidPreviousMs = 0;

  uint8_t tailPowerPercent = 0;
  uint8_t propulsionPowerPercent = 0;
  float speedMps = 0.0f;
  float curveSeverity = 0.0f;
  bool brakeRequested = false;
  VisionBrakePhase brakePhase = VisionBrakePhase::Cruise;
  uint32_t brakePhaseStartedMs = 0;
  bool propulsionReversed = false;
};

struct ActuatorState {
  int leftPropulsionPercent = 0;
  int rightPropulsionPercent = 0;
  float tail1Output = static_cast<float>(robofish::config::lora::kTailFrontCenter);
  float tail2Output = static_cast<float>(robofish::config::lora::kTailRearCenter);
  int throttleOutput = RF_ESC_THROTTLE_NEUTRAL;
  int steeringOutput = RF_ESC_STEERING_CENTER;
  uint32_t lastControlMs = 0;
};

struct MotionControllerState {
  DriveState drive;
  AuxiliaryState auxiliary;
  SafetyState safety;
  DirectionState direction;
  VisionState vision;
  ActuatorState actuator;
  RemoteControlState remote;
  robofish::control::l1::BodyCommand bodyTarget;
  bool bodyActive = false;
  MotionDiagnostics diagnostics;
  // Raw latest samples; seen means history exists, freshness is assessed when
  // making a snapshot. Attitude history remains in diagnostics, one owner.
  robofish::DepthSample depthSample;
  robofish::VelocitySample velocitySample;
  bool depthSeen = false;
  bool velocitySeen = false;
};
