#include "algorithm/PidController.h"

#include <algorithm>
#include <cmath>

namespace robofish::algorithm {

float updatePid(float error, float deltaSeconds, const PidConfig &config,
                PidState &state) {
  state.integral = std::clamp(state.integral + error * deltaSeconds,
                              config.integralMinimum,
                              config.integralMaximum);
  const float derivative =
      (error - state.previousError) / deltaSeconds;
  state.previousError = error;

  if (std::abs(error) <= config.deadband) return 0.0f;
  return config.kp * error + config.ki * state.integral +
         config.kd * derivative;
}

void resetPid(PidState &state) { state = {}; }

}  // namespace robofish::algorithm
