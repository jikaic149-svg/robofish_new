#pragma once
#include <algorithm>

namespace robofish::algorithm {
// 保留两个修正量的比例，同时给已确定的主目标留出余量。
inline float pairedOffsetScale(float base1, float delta1, float low1, float high1,
                              float base2, float delta2, float low2, float high2) {
  const auto scale = [](float base, float delta, float low, float high) {
    if (delta == 0) return 1.0f;
    return std::clamp(((delta > 0 ? high : low) - base) / delta, 0.0f, 1.0f);
  };
  return std::min(scale(base1, delta1, low1, high1), scale(base2, delta2, low2, high2));
}
}
