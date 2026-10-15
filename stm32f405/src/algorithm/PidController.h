#pragma once

namespace robofish::algorithm {

struct PidConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float integralMinimum = 0.0f;
  float integralMaximum = 0.0f;
  float deadband = 0.0f;
};

struct PidState {
  float integral = 0.0f;
  float previousError = 0.0f;
};

float updatePid(float error, float deltaSeconds, const PidConfig &config,
                PidState &state);
void resetPid(PidState &state);

}  // namespace robofish::algorithm
