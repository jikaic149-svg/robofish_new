#include "service/lora/LoraProtocol.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>

namespace robofish::lora {

bool parseLine(const char *line, Command &command) {
  if (!line || !*line) return false;
  long fields[6]{};
  size_t count = 0;
  const char *cursor = line;
  while (*cursor) {
    if (count == 6) return false;
    errno = 0;
    char *end = nullptr;
    const long value = std::strtol(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || value < INT32_MIN || value > INT32_MAX) return false;
    fields[count++] = value;
    if (*end == '\0') break;
    if (*end != ',' || end[1] == '\0') return false;
    cursor = end + 1;
  }
  if (count < 5 || fields[0] != 100 ||
      fields[1] != config::lora::kFishNode ||
      fields[2] < 0 || fields[2] > UINT16_MAX || fields[3] < 0 ||
      fields[3] > UINT16_MAX || fields[4] < 1 || fields[4] > 19) {
    return false;
  }
  // 先完整验证，再更新输出。非法命令不得通过 knob 字段修改控制状态。
  Command parsed;
  parsed.source = static_cast<uint16_t>(fields[2]);
  parsed.sequence = static_cast<uint16_t>(fields[3]);
  parsed.code = static_cast<uint8_t>(fields[4]);
  parsed.hasKnob = count == 6 && fields[5] >= 0;
  if (parsed.hasKnob) {
    parsed.knob = static_cast<uint16_t>(std::clamp(fields[5], 0L, 1000L));
  }
  command = parsed;
  return true;
}

size_t formatAck(char *buffer, size_t capacity, const Command &command,
                 uint32_t acceptedCount) {
  if (!buffer || capacity == 0) return 0;
  const int length = std::snprintf(
      buffer, capacity, "910,%u,%d,%u,%u,%u,%lu\n",
      static_cast<unsigned>(command.source), config::lora::kFishNode,
      static_cast<unsigned>(command.sequence), static_cast<unsigned>(command.source),
      static_cast<unsigned>(command.code),
      static_cast<unsigned long>(acceptedCount));
  if (length < 0 || static_cast<size_t>(length) >= capacity) {
    buffer[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(length);
}

ParseResult LineDecoder::consume(uint8_t byte, Command &command) {
  if (byte == '\r' || byte == 0xff) return ParseResult::Incomplete;
  if (byte == '\n') {
    if (discarding_) {
      reset();
      return ParseResult::Rejected;
    }
    if (length_ == 0) return ParseResult::Incomplete;
    line_[length_] = '\0';
    const bool accepted = parseLine(line_, command);
    reset();
    return accepted ? ParseResult::Accepted : ParseResult::Rejected;
  }
  if (discarding_) return ParseResult::Incomplete;
  if (byte < ' ' || byte > '~' || length_ == config::lora::kMaxLineLength) {
    discarding_ = true;
    return ParseResult::Incomplete;
  }
  line_[length_++] = static_cast<char>(byte);
  return ParseResult::Incomplete;
}

void LineDecoder::reset() {
  length_ = 0;
  discarding_ = false;
}

}  // namespace robofish::lora
