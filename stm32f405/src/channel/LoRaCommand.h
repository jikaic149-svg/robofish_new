#pragma once

#include <stdint.h>

namespace robofish::lora {

// 协议边界校验后交给唯一控制器的文本遥控命令；不是线上二进制结构体。
struct Command {
  uint16_t sequence = 0;
  uint8_t code = 0;
  bool hasKnob = false;
  uint16_t knob = 0;
  uint16_t source = 0;  // Existing third wire field; sequence is tracked per source.
};

}  // namespace robofish::lora
