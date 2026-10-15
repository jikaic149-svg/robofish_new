#pragma once
#include <cmath>
#include <stdint.h>

namespace robofish::algorithm {
inline constexpr double kTwoPi = 6.28318530717958647692;
struct GaitPhase {
  double radians = 0;
  uint32_t updatedMs = 0;
  bool initialized = false;
};
// 单一输出任务持有状态；无硬件/RTOS依赖，uint32 差值支持时间回绕。
inline double advanceGait(GaitPhase &state, uint32_t nowMs, double frequencyHz) {
  if (!state.initialized) {
    state.radians = std::remainder(nowMs * 0.001 * frequencyHz * kTwoPi, kTwoPi);
    state.initialized = true;
  } else {
    state.radians = std::remainder(state.radians +
        (nowMs - state.updatedMs) * 0.001 * frequencyHz * kTwoPi, kTwoPi);
  }
  state.updatedMs = nowMs;
  return state.radians;
}
}
