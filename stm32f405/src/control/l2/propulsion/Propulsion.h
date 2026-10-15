#pragma once

#include "control/l2/Types.h"

namespace robofish::control::l2::propulsion {

struct Output {
  int targetLeft = 0;
  int targetRight = 0;
  int left = 0;
  int right = 0;
  float common = 0;
  float differential = 0;
  uint32_t limits = 0;
};

Output allocate(const Request &request, const PropulsionRecipe &recipe,
                const ActuatorFrame &lastCommitted, uint32_t slewElapsedMs);

}  // namespace robofish::control::l2::propulsion
