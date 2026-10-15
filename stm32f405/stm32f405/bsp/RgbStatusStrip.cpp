#include "bsp/RgbStatusStrip.h"

#include "main.h"

namespace {
constexpr uint16_t kZeroDuty = 29;  // 0.35 us high at 84 MHz.
constexpr uint16_t kOneDuty = 58;   // 0.69 us high at 84 MHz.
constexpr uint8_t kBrightness = 40;
volatile bool gWs2812Busy = false;

uint8_t scale(uint8_t value) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * kBrightness + 127U) / 255U);
}

void appendByte(uint16_t *words, size_t &offset, uint8_t value) {
  for (int bit = 7; bit >= 0; --bit)
    words[offset++] = (value & (1U << bit)) ? kOneDuty : kZeroDuty;
}
}  // namespace

RgbStatusStrip::RgbStatusStrip() = default;

bool RgbStatusStrip::begin() {
  if (htim4.Instance == nullptr) return false;
  ready_.store(true);
  return setColor(0, 0, 0);
}

bool RgbStatusStrip::setColor(uint8_t red, uint8_t green, uint8_t blue) {
  if (!ready_.load() || gWs2812Busy) return false;
  size_t offset = 0;
  const uint8_t grb[3] = {scale(green), scale(red), scale(blue)};
  for (size_t pixel = 0; pixel < 8; ++pixel)
    for (uint8_t byte : grb) appendByte(dmaWords_.data(), offset, byte);
  while (offset < dmaWords_.size()) dmaWords_[offset++] = 0;
  gWs2812Busy = true;
  if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3,
                            reinterpret_cast<uint32_t *>(dmaWords_.data()),
                            dmaWords_.size()) != HAL_OK) {
    gWs2812Busy = false;
    return false;
  }
  return true;
}

bool RgbStatusStrip::ready() const { return ready_.load(); }

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *timer) {
  if (timer == &htim4) {
    HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_3);
    gWs2812Busy = false;
  }
}
