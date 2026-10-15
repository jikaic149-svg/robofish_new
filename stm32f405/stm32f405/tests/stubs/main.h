#pragma once
#include <cstdint>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };
struct TIM_TypeDef {};
struct USART_TypeDef {};
inline TIM_TypeDef tim1, tim2, tim3, tim4, tim5;
inline USART_TypeDef usart1;
#define TIM1 (&tim1)
#define TIM2 (&tim2)
#define TIM3 (&tim3)
#define TIM4 (&tim4)
#define TIM5 (&tim5)
#define USART1 (&usart1)
// STM32F4 HAL encodes CH1 as zero, not as an invalid channel.
constexpr uint32_t TIM_CHANNEL_1 = 0;
constexpr uint32_t TIM_CHANNEL_2 = 4;
constexpr uint32_t TIM_CHANNEL_3 = 8;
constexpr uint32_t TIM_CHANNEL_4 = 12;
constexpr uint32_t TIM_CHANNEL_ALL = 24;
struct TIM_HandleTypeDef {
  TIM_TypeDef *Instance = nullptr;
  uint32_t compare[4]{};
};
enum HAL_UART_StateTypeDef {
  HAL_UART_STATE_READY = 0x20,
  HAL_UART_STATE_BUSY_TX = 0x21,
  HAL_UART_STATE_BUSY_RX = 0x22,
  HAL_UART_STATE_BUSY_TX_RX = 0x23,
  HAL_UART_STATE_ERROR = 0xE0
};
struct UART_HandleTypeDef {
  USART_TypeDef *Instance = nullptr;
  HAL_UART_StateTypeDef gState = HAL_UART_STATE_READY;
  HAL_UART_StateTypeDef RxState = HAL_UART_STATE_READY;
};
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5;
extern UART_HandleTypeDef huart1;
inline HAL_StatusTypeDef pwmResult = HAL_OK, txResult = HAL_OK, rxResult = HAL_OK;
inline unsigned receiveAborts = 0, clearedReceiveErrors = 0;
#define __HAL_UART_CLEAR_OREFLAG(uart) (++clearedReceiveErrors)
inline uint8_t *rxByte = nullptr;
inline const uint8_t *txBytes = nullptr;
inline uint16_t txLength = 0, waveformLength = 0;
inline const uint16_t *waveform = nullptr;
inline unsigned aborts = 0, rxArms = 0;
inline bool receiveImmediately = false;
namespace robofish::bsp { void onUartRxComplete(); }
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare[(channel) / 4] = (value))
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t) { return pwmResult; }
inline HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *, uint32_t,
                                              uint32_t *data, uint16_t length) {
  waveform = reinterpret_cast<uint16_t *>(data);
  waveformLength = length;
  return pwmResult;
}
inline HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t) {
  if (rxResult != HAL_OK) return rxResult;
  rxByte = data;
  uart->RxState = HAL_UART_STATE_BUSY_RX;
  ++rxArms;
  if (receiveImmediately) {
    receiveImmediately = false;
    *data = 127;
    robofish::bsp::onUartRxComplete();
  }
  return HAL_OK;
}
inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                             const uint8_t *data, uint16_t length) {
  if (txResult != HAL_OK) return txResult;
  txBytes = data; txLength = length;
  uart->gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}
inline HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *uart) {
  return static_cast<HAL_UART_StateTypeDef>(uart->gState | uart->RxState);
}
inline HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *uart) {
  ++aborts;
  uart->gState = uart->RxState = HAL_UART_STATE_READY;
  return HAL_OK;
}

inline HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *uart) {
  ++receiveAborts;
  uart->RxState = HAL_UART_STATE_READY;
  return HAL_OK;
}
