#pragma once

#include <atomic>

#include "bsp/LoRaUart.h"
#include "service/lora/LoraProtocol.h"
#include "config/ControlConfig.h"
#include "channel/LoRaLinkStatus.h"

namespace robofish::lora {

class LoRaService {
 public:
  using Submit = bool (*)(const Command &, void *);
  LoRaService(Submit submit, void *context, config::radio::Timing timing = config::radio::kTiming)
      : submit_(submit), context_(context), timing_(timing) {}
  bool begin();
  void poll(uint32_t nowMs);
  void stop();
  // 仅所属主循环读取；HTTP跨任务诊断使用MotionController的带锁快照。
  const LinkStatus &status() const { return status_; }

 private:
  void handle(const Command &command, uint32_t nowMs);
  void advanceTransaction(uint32_t nowMs);
  void scheduleAck(const Command &command, uint32_t acceptedCount, uint32_t nowMs);
  bool timingValid() const;
  void restrictTx();
  bsp::LoRaUart uart_;
  LineDecoder decoder_;
  Submit submit_;
  void *context_;
  LinkStatus status_;
  config::radio::Timing timing_;
  struct Recent { Command command{}; uint32_t acceptedCount = 0; bool valid = false; };
  Recent recent_[config::radio::kRecentTransactionCount]{};
  unsigned nextRecent_ = 0;
  struct SourceSequence { uint16_t source = 0, sequence = 0; bool valid = false; };
  SourceSequence sources_[config::radio::kMaximumSources]{};
  std::atomic_bool stopRequested_{false};
  uint32_t ackDueMs_ = 0;
  uint32_t replyDeadlineMs_ = 0;
  uint32_t phaseDeadlineMs_ = 0;
  char txLine_[64]{};
  size_t txLength_ = 0;
};

}  // namespace robofish::lora
