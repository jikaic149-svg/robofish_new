#include "service/lora/LoRaService.h"

namespace robofish::lora {
namespace {
bool due(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now-deadline) >= 0; }
bool same(const Command &a, const Command &b) {
  return a.source == b.source && a.sequence == b.sequence && a.code == b.code && a.hasKnob == b.hasKnob &&
      (!a.hasKnob || a.knob == b.knob);
}
}

bool LoRaService::timingValid() const {
  const uint64_t releaseUs = static_cast<uint64_t>(timing_.rfReleaseBudgetMs)*1000;
  const uint64_t total = static_cast<uint64_t>(timing_.replyDelayMs) +
      timing_.uartDrainTimeoutMs + timing_.rfReleaseBudgetMs;
  return timing_.ackTxEnabled && timing_.peerWindowsVerified && timing_.moduleConfigurationVerified &&
      timing_.packetBytes >= 30 && timing_.packetBytes <= 230 && timing_.packetIdleUs > 0 &&
      timing_.uartDrainTimeoutMs > 0 && releaseUs >= timing_.packetIdleUs &&
      timing_.rfReleaseBudgetMs > 0 && timing_.replyWindowMs > total &&
      timing_.replyWindowMs < 0x80000000u;
}

bool LoRaService::begin() {
  stopRequested_.store(false);
  decoder_.reset(); status_ = {};
  for (auto &source : sources_) source = {};
  txLength_ = 0;
  for (auto &entry : recent_) entry = {};
  nextRecent_ = 0;
  const bool ready = submit_ && uart_.begin();
  status_.txEnabled = ready && timingValid();
  return ready;
}

void LoRaService::restrictTx() {
  ++status_.txErrors;
  status_.txEnabled = false;
  status_.phase = LinkPhase::RestrictedReceive;
  txLength_ = 0;
}

void LoRaService::scheduleAck(const Command &command, uint32_t count, uint32_t now) {
  if (!status_.txEnabled) { ++status_.ackSuppressed; return; }
  txLength_ = formatAck(txLine_, sizeof(txLine_), command, count);
  if (!txLength_ || txLength_ > timing_.packetBytes) { restrictTx(); return; }

  ackDueMs_ = now + timing_.replyDelayMs;
  replyDeadlineMs_ = now + timing_.replyWindowMs;
  status_.phase = LinkPhase::ReplyWait;
}

void LoRaService::handle(const Command &command, uint32_t now) {
  if (stopRequested_.load()) return;
  for (const auto &entry : recent_) {
    if (!entry.valid || entry.command.source != command.source ||
        entry.command.sequence != command.sequence) continue;
    if (!same(entry.command, command)) { ++status_.rejected; return; }
    ++status_.duplicates;
    // 重复不延长进行中回复；事务结束后的重传可重发原接受结果。
    if (status_.phase == LinkPhase::Receive) scheduleAck(command, entry.acceptedCount, now);
    return;
  }
  SourceSequence *source = nullptr, *empty = nullptr;
  for (auto &entry : sources_) {
    if (entry.valid && entry.source == command.source) source = &entry;
    else if (!entry.valid && !empty) empty = &entry;
  }
  if (source && !sequenceAfter(command.sequence, source->sequence)) {
    ++status_.stale; ++status_.rejected;
    return;
  }
  if (!source) source = empty;
  if (!source) { ++status_.rejected; return; }
  const bool stop = command.code == 1 || command.code == 17;
  const bool inFlight = status_.phase != LinkPhase::Receive && status_.phase != LinkPhase::RestrictedReceive;
  if (inFlight && !stop) { ++status_.rejected; return; }
  if (!submit_ || !submit_(command, context_)) { ++status_.rejected; return; }
  *source = {command.source, command.sequence, true};
  status_.lastSource = command.source;
  ++status_.accepted;
  status_.lastCommandMs = now; status_.lastSequence = command.sequence; status_.lastCode = command.code;
  recent_[nextRecent_] = {command, status_.accepted, true};
  nextRecent_ = (nextRecent_+1) % config::radio::kRecentTransactionCount;
  // STOP先执行；仅可取消尚未交给UART的普通ACK。空口/排空帧不可抢断。
  if (inFlight && (status_.phase == LinkPhase::TxDrain || status_.phase == LinkPhase::AirRelease)) {
    ++status_.ackSuppressed; return;
  }
  scheduleAck(command, status_.accepted, now);
}

void LoRaService::advanceTransaction(uint32_t now) {
  switch (status_.phase) {
    case LinkPhase::Receive: case LinkPhase::RestrictedReceive: return;
    case LinkPhase::ReplyWait:
      if (due(now, replyDeadlineMs_)) {
        ++status_.missedReplyWindows; txLength_ = 0; status_.phase = LinkPhase::Receive;
      } else if (due(now, ackDueMs_)) status_.phase = LinkPhase::TxSubmit;
      return;
    case LinkPhase::TxSubmit: {
      // 为排空与无线尾部预留完整预算；错窗不迟发。
      const uint32_t remaining = replyDeadlineMs_-now;
      const uint64_t needed = static_cast<uint64_t>(timing_.uartDrainTimeoutMs)+timing_.rfReleaseBudgetMs;
      if (due(now, replyDeadlineMs_) || remaining <= needed) {
        ++status_.missedReplyWindows; txLength_ = 0; status_.phase = LinkPhase::Receive; return;
      }
      const auto sent = uart_.tryWriteFrame(txLine_, txLength_);
      if (sent.error || (sent.bytes > 0 && static_cast<size_t>(sent.bytes) != txLength_)) { restrictTx(); return; }
      if (!sent.bytes) return;

      phaseDeadlineMs_ = now + timing_.uartDrainTimeoutMs;
      status_.phase = LinkPhase::TxDrain;
      return;
    }
    case LinkPhase::TxDrain: {
      const auto state = uart_.txState();
      if (state == bsp::UartTxState::Error || due(now, phaseDeadlineMs_)) { restrictTx(); return; }
      if (state == bsp::UartTxState::Idle) {
        phaseDeadlineMs_ = now + timing_.rfReleaseBudgetMs;
        status_.phase = LinkPhase::AirRelease;
      }
      return;
    }
    case LinkPhase::AirRelease:
      if (due(now, phaseDeadlineMs_)) {
        ++status_.acknowledgements; ++status_.releasedByBudget;
        txLength_ = 0; status_.phase = LinkPhase::Receive;
      }
      return;
  }
}

void LoRaService::poll(uint32_t now) {
  if (stopRequested_.load()) {
    uart_.stop(); decoder_.reset();
    txLength_ = 0; status_.txEnabled = false; status_.phase = LinkPhase::RestrictedReceive;
    return;
  }
  // 先结算已到期的旧事务，避免合法下一帧被旧phase误拒；不提前发送ACK。
  if (status_.phase == LinkPhase::AirRelease && due(now, phaseDeadlineMs_))
    advanceTransaction(now);
  if ((status_.phase == LinkPhase::ReplyWait || status_.phase == LinkPhase::TxSubmit) && due(now, replyDeadlineMs_)) {
    ++status_.missedReplyWindows;
    txLength_ = 0; status_.phase = LinkPhase::Receive;
  }
  uint8_t bytes[config::radio::kReadBudgetBytes];
  const int count = uart_.read(bytes, sizeof(bytes));
  if (count < 0) {
    ++status_.receiveErrors;
    decoder_.discardUntilNewline();  // Never splice command bytes across a lost byte.
  }
  for (int i=0; i<count; ++i) {
    Command command;
    const auto parsed = decoder_.consume(bytes[i], command);
    if (parsed == ParseResult::Accepted) handle(command, now);
    else if (parsed == ParseResult::Rejected) ++status_.rejected;
  }
  advanceTransaction(now);
}
void LoRaService::stop() { stopRequested_.store(true); }
}
