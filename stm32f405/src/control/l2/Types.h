#pragma once

#include <stdint.h>

#include "algorithm/GaitPhase.h"
#include "channel/ActuatorFrame.h"
#include "channel/MotionTypes.h"

namespace robofish::control::l2 {

// Body requests are percentages, not measured forces, rates or attitude targets.
// +surge is forward, +yaw is right; pitch/roll mapping is explicitly calibrated.
struct Request {
  DriveMode mode = DriveMode::Idle;
  float surgePercent = 0;
  float yawPercent = 0;
  float pitchPercent = 0;
  float rollPercent = 0;
  bool enabled = false;
  bool allowFlowAssist = false;
  bool immediate = false;  // Existing immediate stop/failsafe path; bypass slew only.
  float outputScale = 1;
  // Preserve independent tail/prop drive demands from existing upstream controls.
  // Nodes do not know whether the caller is a radio, vision or manual source.
  bool independentDrives = false;
  float tailDrivePercent = 0;
  float propulsionDrivePercent = 0;
  bool tailEnabled = true;
  bool propulsionEnabled = true;
  bool independentTailYaw = false;
  float tailYawPercent = 0;
};

struct JointGeometry {
  float centerDegrees = 0;
  float minimumDegrees = 0;
  float maximumDegrees = 0;
};
struct Geometry {
  JointGeometry tailFront;
  JointGeometry tailRear;
  JointGeometry pectoralLeft;
  JointGeometry pectoralRight;
};

struct TailRecipe {
  float amplitudeDegrees = 0;
  float frequencyHz = 0;
  float rearAmplitudeRatio = 0;
  float phaseLagRad = 0;  // rear = sin(sharedPhase - lag)
  float turnBiasDegrees = 0;
  float staticSteeringDegrees = 0;
  float rateDegreesPerSecond = 0;
};
struct PropulsionRecipe {
  float commandScale = 1;
  float yawGain = 1;
  float ratePercentPerSecond = 0;
};
struct PectoralRecipe {
  // Offsets from each geometric center, in the existing absolute-angle encoding.
  float leftPositivePitchDegrees = 0;
  float rightPositivePitchDegrees = 0;
  float leftNegativePitchDegrees = 0;
  float rightNegativePitchDegrees = 0;
  float leftRollDegrees = 0;
  float rightRollDegrees = 0;
  float leftYawDegrees = 0;
  float rightYawDegrees = 0;
  float leftSurgeTrimDegrees = 0;
  float rightSurgeTrimDegrees = 0;
  float leftTailTrimDegrees = 0;
  float rightTailTrimDegrees = 0;
  float rateDegreesPerSecond = 0;
};
struct Recipe {
  TailRecipe tail;
  PropulsionRecipe propulsion;
  PectoralRecipe pectoral;
  float flowMinimumPercent = 0;
  float diveDragCompensationPercent = 0;
  uint32_t maximumSlewElapsedMs = 0;
};

enum Limit : uint32_t {
  PropulsionLimited = 1,
  TailLimited = 2,
  PectoralLimited = 4,
  ReverseUnsupported = 8,
  InvalidInput = 16,
  TailRateLimited = 32,
  PropulsionRateLimited = 64,
  PectoralRateLimited = 128
};

// The clock is the only internal state. Per-channel successful nominal outputs
// are supplied separately by the owner after L3 reports its written mask.
struct State {
  algorithm::GaitPhase gait;
};
struct Result {
  ActuatorFrame targetFrame;     // Geometrically feasible, before per-tick slew.
  ActuatorFrame candidateFrame;  // Nominal output offered to L3, never committed here.
  double phaseRad = 0;
  float commonPropulsionPercent = 0;
  float differentialPropulsionPercent = 0;
  float tailLoad = 0;
  float pectoralCorrectionScale = 1;
  uint32_t limitedReasons = 0;
  bool unsupportedReverse = false;
};

}  // namespace robofish::control::l2
