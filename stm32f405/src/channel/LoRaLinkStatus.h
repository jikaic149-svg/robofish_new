#pragma once
#include <stdint.h>
namespace robofish::lora {
enum class LinkPhase : uint8_t { Receive, ReplyWait, TxSubmit, TxDrain, AirRelease, RestrictedReceive };
struct LinkStatus {
  uint32_t accepted = 0;
  uint32_t rejected = 0;
  uint32_t duplicates = 0;
  uint32_t acknowledgements = 0;
  uint32_t lastCommandMs = 0;
  uint16_t lastSequence = 0;
  uint16_t lastSource = 0;
  uint32_t stale = 0, receiveErrors = 0;
  uint8_t lastCode = 0;
  LinkPhase phase = LinkPhase::Receive;
  bool txEnabled = false;
  uint32_t ackSuppressed = 0;
  uint32_t missedReplyWindows = 0;
  uint32_t txErrors = 0;
  uint32_t releasedByBudget = 0; // 不表示测得RF完成。
};

}
