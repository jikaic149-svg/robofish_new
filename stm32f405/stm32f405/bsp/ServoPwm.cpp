#include "bsp/ServoPwm.h"

#include <algorithm>

#include "main.h"
#include "bsp/RuntimeSafety.h"

namespace {
TIM_HandleTypeDef *timerFor(robofish::bsp::PwmTimer timer) {
  using robofish::bsp::PwmTimer;
  switch (timer) {
    case PwmTimer::kTim1: return &htim1;
    case PwmTimer::kTim2: return &htim2;
    case PwmTimer::kTim3: return &htim3;
    case PwmTimer::kTim5: return &htim5;
  }
  return nullptr;
}

uint32_t channelFor(uint32_t logicalChannel) {
  switch (logicalChannel) {
    case 1: return TIM_CHANNEL_1;
    case 2: return TIM_CHANNEL_2;
    case 3: return TIM_CHANNEL_3;
    case 4: return TIM_CHANNEL_4;
    default: return TIM_CHANNEL_ALL;
  }
}
}  // namespace

ServoPwm::ServoPwm() = default;

bool ServoPwm::configureTimers() {
  // The board configures all servo timers at 1 MHz / 20,000 counts. Starting is
  // deliberately deferred until an individual output owns its channel.
  return htim1.Instance != nullptr && htim2.Instance != nullptr &&
         htim3.Instance != nullptr && htim5.Instance != nullptr;
}

bool ServoPwm::begin(robofish::bsp::BoardPin, uint32_t logicalChannel,
                     robofish::bsp::PwmTimer timer, int minPulseUs,
                     int maxPulseUs) {
  TIM_HandleTypeDef *handle = timerFor(timer);
  const uint32_t channel = channelFor(logicalChannel);
  if (!handle || channel == TIM_CHANNEL_ALL || minPulseUs < 0 ||
      maxPulseUs < minPulseUs) return false;
  timer_ = handle;
  channel_ = channel;
  minPulseUs_ = minPulseUs;
  maxPulseUs_ = maxPulseUs;
  __HAL_TIM_SET_COMPARE(handle, channel, static_cast<uint32_t>((minPulseUs + maxPulseUs) / 2));
  initialized_ = HAL_TIM_PWM_Start(handle, channel) == HAL_OK;
  return initialized_;
}

bool ServoPwm::writeAngle(int angle) {
  angle = std::clamp(angle, 0, 180);
  return writeMicroseconds(minPulseUs_ +
      angle * (maxPulseUs_ - minPulseUs_) / 180);
}

bool ServoPwm::writeMicroseconds(int pulseUs) {
  if (!initialized_ || robofish::bsp::faultLatched()) return false;
  pulseUs = std::clamp(pulseUs, minPulseUs_, maxPulseUs_);
  auto *handle = static_cast<TIM_HandleTypeDef *>(timer_);
  // TIM1/2/3/5 use 1 MHz, so CCR is directly in microseconds.
  __HAL_TIM_SET_COMPARE(handle, channel_, static_cast<uint32_t>(pulseUs));
  return true;
}
