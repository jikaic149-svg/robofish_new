#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config/RuntimeConfig.h"
#include "channel/LoRaCommand.h"

namespace robofish::lora {

enum class ParseResult : uint8_t { Incomplete, Accepted, Rejected };

// RFC 1982 half-range ordering; exact half-range distance is rejected.
inline bool sequenceAfter(uint16_t incoming, uint16_t previous) {
  const uint16_t distance = static_cast<uint16_t>(incoming - previous);
  return distance != 0 && distance < 0x8000U;
}

bool parseLine(const char *line, Command &command);
size_t formatAck(char *buffer, size_t capacity, const Command &command,
                 uint32_t acceptedCount);

// 固定缓冲且逐字节推进；超长/非法行丢弃到换行，不能把尾部当新命令。
class LineDecoder {
 public:
  ParseResult consume(uint8_t byte, Command &command);
  void reset();
  void discardUntilNewline() { length_ = 0; discarding_ = true; }

 private:
  char line_[config::lora::kMaxLineLength + 1]{};
  size_t length_ = 0;
  bool discarding_ = false;
};

}  // namespace robofish::lora
