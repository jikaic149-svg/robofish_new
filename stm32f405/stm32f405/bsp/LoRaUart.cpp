#include "bsp/LoRaUart.h"

#include <algorithm>

#include "main.h"

namespace robofish::bsp {
namespace {
LoRaUart *gOwner = nullptr;
}

bool LoRaUart::begin() {
  if (ready_) return true;
  if (huart1.Instance != USART1) return false;
  receiveDiscontinuity_.store(false);
  readIndex_.store(0);
  writeIndex_.store(0);
  gOwner = this;
  ready_ = true;
  if (HAL_UART_Receive_IT(&huart1, &rxByte_, 1) != HAL_OK) {
    ready_ = false;
    gOwner = nullptr;
    return false;
  }
  return true;
}

int LoRaUart::read(uint8_t *buffer, size_t capacity) {
  if (!ready_ || !buffer || !capacity) return 0;
  // A failed rearm is retried by RadioPoll; never abort a concurrent TX DMA.
  if (huart1.RxState == HAL_UART_STATE_READY)
    (void)HAL_UART_Receive_IT(&huart1, &rxByte_, 1);
  if (receiveDiscontinuity_.exchange(false, std::memory_order_acq_rel)) {
    readIndex_.store(writeIndex_.load(std::memory_order_acquire), std::memory_order_release);
    return -1;
  }
  size_t count = 0;
  size_t read = readIndex_.load(std::memory_order_relaxed);
  const size_t end = writeIndex_.load(std::memory_order_acquire);
  while (read != end && count < capacity) {
    buffer[count++] = ring_[read];
    read = (read + 1) % kRingBytes;
  }
  readIndex_.store(read, std::memory_order_release);
  if (receiveDiscontinuity_.exchange(false, std::memory_order_acq_rel)) {
    readIndex_.store(writeIndex_.load(std::memory_order_acquire), std::memory_order_release);
    return -1;
  }
  return static_cast<int>(count);
}

int LoRaUart::writeAvailable(const char *data, size_t length) {
  if (!ready_ || !data || !length || length > 0x7fff) return 0;
  const HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(
      &huart1, reinterpret_cast<const uint8_t *>(data),
      static_cast<uint16_t>(length));
  return result == HAL_OK ? static_cast<int>(length) : result == HAL_BUSY ? 0 : -1;
}

UartTxState LoRaUart::txState() const {
  if (!ready_) return UartTxState::Error;
  const HAL_UART_StateTypeDef state = huart1.gState;
  if (state == HAL_UART_STATE_READY) return UartTxState::Idle;
  if (state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX)
    return UartTxState::Busy;
  return UartTxState::Error;
}

UartWriteResult LoRaUart::tryWriteFrame(const char *data, size_t length) {
  if (!ready_ || !data || !length || length > 64) return {0, true};
  const auto state = txState();
  if (state != UartTxState::Idle) return {0, state == UartTxState::Error};
  const int written = writeAvailable(data, length);
  return {written > 0 ? written : 0, written < 0};
}

void LoRaUart::stop() {
  if (ready_) HAL_UART_Abort(&huart1);
  if (gOwner == this) gOwner = nullptr;
  ready_ = false;
}

void LoRaUart::onRxByte(uint8_t byte) {
  const size_t write = writeIndex_.load(std::memory_order_relaxed);
  const size_t next = (write + 1) % kRingBytes;
  if (next != readIndex_.load(std::memory_order_acquire)) {
    ring_[write] = byte;
    writeIndex_.store(next, std::memory_order_release);
  } else receiveDiscontinuity_.store(true, std::memory_order_release);
  if (ready_ && HAL_UART_Receive_IT(&huart1, &rxByte_, 1) != HAL_OK)
    receiveDiscontinuity_.store(true, std::memory_order_release);
}

void LoRaUart::onUartRxComplete() { onRxByte(rxByte_); }

void LoRaUart::onUartError() {
  if (!ready_) return;
  receiveDiscontinuity_.store(true, std::memory_order_release);
  // Receive-only abort is immediate in IT mode and leaves the TX DMA untouched.
  (void)HAL_UART_AbortReceive_IT(&huart1);
  __HAL_UART_CLEAR_OREFLAG(&huart1); // SR/DR read also clears PE/FE/NE.
  (void)HAL_UART_Receive_IT(&huart1, &rxByte_, 1);
}

void onUartRxComplete() {
  if (gOwner) gOwner->onUartRxComplete();
}
void onUartError() {
  if (gOwner) gOwner->onUartError();
}
}  // namespace robofish::bsp
