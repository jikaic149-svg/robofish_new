#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <stdint.h>

class RgbStatusStrip {
 public:
  RgbStatusStrip();
  bool begin();
  bool setColor(uint8_t red, uint8_t green, uint8_t blue);
  bool ready() const;

 private:
  static constexpr size_t kPixelCount = 8;
  // HAL's PWM-DMA API takes a uint32_t pointer even when the DMA transfer is
  // half-word wide.  Keep the half-word waveform but give its base 32-bit
  // alignment so the pointer conversion is well-defined on Cortex-M.
  alignas(uint32_t) std::array<uint16_t, kPixelCount * 24 + 64> dmaWords_{};
  std::atomic_bool ready_{false};
};
