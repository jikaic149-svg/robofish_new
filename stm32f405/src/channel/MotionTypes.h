#pragma once

#include <stdint.h>

// 机器鱼的推进方式：待机、仿生鱼尾、螺旋桨和混合推进。
enum class DriveMode : uint8_t { Idle = 0, Bionic, Prop, Hybrid };

// 上层命令使用的离散运动方向。
enum class MotionAction : uint8_t { Stopped = 0, Forward, Backward, Left, Right };

// ---------------------------------------------------------------------------
// 下一目标：外部模块希望 MotionController 接下来执行的动作。
// ---------------------------------------------------------------------------

// 人工控制和 Partner 共用的低频离散运动目标。
struct MotionCommand {
  // 目标动作，以及直行功率和转向强度，百分比范围均为 0～100。
  MotionAction action = MotionAction::Stopped;
  uint8_t powerPercent = 55;
  uint8_t strengthPercent = 40;
};

// HTTP 与 UDP 共用的高频连续运动目标。控制器内部约定正值向右。
// 新目标覆盖旧目标，不排队执行过期的视觉帧。
struct VisionSetpoint {
  // sequence 必须持续递增，用于拒绝重复或倒序到达的控制目标。
  uint32_t sequence = 0;
  uint8_t powerPercent = 0;
  int16_t steeringPercent = 0;
  bool brakeRequested = false;
  uint32_t controlGeneration = 0; // Obtained when local control is explicitly authorized.
};

// ---------------------------------------------------------------------------
// 当前状态：MotionController 已经接受并实际应用的运行状态快照。
// ---------------------------------------------------------------------------

struct MotionStatus {
  // 当前控制状态。
  DriveMode mode;
  MotionAction action;
  uint8_t powerPercent;
  uint8_t turnStrengthPercent;
  int pitchAngle;
  bool failsafeActive;
  // Tail fields are submitted nominal angles. Legacy ESC fields are display
  // projections only; actual electrical commands are in MotionDiagnostics.
  int tail1Output;
  int tail2Output;
  int escThrottleOutput;
  int escSteeringOutput;
  uint32_t visionAppliedSequence;
};

// ---------------------------------------------------------------------------
// 处理结果：需要向 Web 或视觉服务说明拒绝原因的少量返回类型。
// 普通 MotionCommand 仍然只返回 bool。
// ---------------------------------------------------------------------------

// 视觉目标的接收结果；序列号用于识别重复或过期数据。
enum class VisionControlResult : uint8_t { Accepted, StaleSequence, Rejected };
