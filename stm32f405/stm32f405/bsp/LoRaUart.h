#pragma once

#include <stddef.h>
#include <stdint.h>
#include <atomic>

namespace robofish::bsp {
enum class UartTxState : uint8_t { Idle, Busy, Error };
struct UartWriteResult { int bytes = 0; bool error = false; };

class LoRaUart {
 public:
  bool begin();
  int read(uint8_t *buffer, size_t capacity); // -1: stream discontinuity, discard partial line.
  int writeAvailable(const char *data, size_t length);
  UartTxState txState() const;
  UartWriteResult tryWriteFrame(const char *data, size_t length);
  void stop();
  bool ready() const { return ready_; }
  void onRxByte(uint8_t byte);
  void onUartRxComplete();
  void onUartError();

 private:
  static constexpr size_t kRingBytes = 512;
  std::atomic_size_t readIndex_{0};
  std::atomic_size_t writeIndex_{0};
  uint8_t ring_[kRingBytes]{};
  uint8_t rxByte_ = 0;
  bool ready_ = false;
  std::atomic_bool receiveDiscontinuity_{false};
};

// Called by the Cube/HAL UART IRQ callback in main.cpp.
void onUartRxComplete();
void onUartError();
}  // namespace robofish::bsp
