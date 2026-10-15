#pragma once
#include <stdint.h>
#include "channel/ActuatorFrame.h"

namespace robofish {
enum FrameChannel : uint8_t {
  TailFront = 1, TailRear = 2, PectoralLeft = 4, PectoralRight = 8,
  PropulsionLeft = 16, PropulsionRight = 32, AllFrameChannels = 63
};
struct FrameApplyResult {
  uint8_t writtenMask = 0; // 仅表示驱动API接受写入，不表示实际位置反馈。
  // Signed electrical ESC commands after installation mapping and ONE voltage
  // compensation; meaningful only for channels in writtenMask. Never feed
  // these back as nominal propulsion requests in the next L2 calculation.
  int16_t leftPropulsionPercent = 0;
  int16_t rightPropulsionPercent = 0;
  bool complete() const { return writtenMask == AllFrameChannels; }
};
inline void mergeWrittenFrame(ActuatorFrame &last, const ActuatorFrame &attempt, uint8_t mask) {
  if (mask & TailFront) last.tailFrontDegrees = attempt.tailFrontDegrees;
  if (mask & TailRear) last.tailRearDegrees = attempt.tailRearDegrees;
  if (mask & PectoralLeft) last.pectoralLeftDegrees = attempt.pectoralLeftDegrees;
  if (mask & PectoralRight) last.pectoralRightDegrees = attempt.pectoralRightDegrees;
  if (mask & PropulsionLeft) last.leftPropulsionPercent = attempt.leftPropulsionPercent;
  if (mask & PropulsionRight) last.rightPropulsionPercent = attempt.rightPropulsionPercent;
  if (mask == AllFrameChannels) last.flowAssisted = attempt.flowAssisted;
}
}
