#pragma once

#include "control/l2/Types.h"

namespace robofish::control::l2::tail {

struct Output {
  float targetFront = 0;
  float targetRear = 0;
  float front = 0;
  float rear = 0;
  float load = 0;
  uint32_t limits = 0;
};

Output allocate(const Request &request, const TailRecipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                double phaseRad, uint32_t slewElapsedMs);

}  // namespace robofish::control::l2::tail
