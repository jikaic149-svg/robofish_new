#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "main.h"
#include "bsp/ServoPwm.h"
#include "bsp/LoRaUart.h"
#include "bsp/RgbStatusStrip.h"

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5;
UART_HandleTypeDef huart1;
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *);
static unsigned checks = 0;
static void check(bool value, const char *name) {
  ++checks;
  if (!value) { std::fprintf(stderr, "FAIL: %s\n", name); std::exit(1); }
}

int main() {
  using namespace robofish::bsp;
  check(!ServoPwm::configureTimers(), "uninitialized timers rejected");
  htim1.Instance = TIM1; htim2.Instance = TIM2; htim3.Instance = TIM3;
  htim4.Instance = TIM4; htim5.Instance = TIM5;
  check(ServoPwm::configureTimers(), "all PWM timers available");
  const ServoResource resources[] = {kTailServo1, kTailServo2, kPectoralLeftServo,
      kPectoralRightServo, kPropulsionLeft, kPropulsionRight};
  std::array<ServoPwm, 6> outputs;
  for (unsigned i = 0; i < outputs.size(); ++i) {
    const auto &r = resources[i];
    check(outputs[i].begin(r.gpio, r.channel, r.timer, r.minimumPulseUs, r.maximumPulseUs),
          "every physical output, including all CH1 instances, starts");
  }
  check(outputs[0].writeAngle(90) && htim5.compare[1] == 1500, "upper tail PA1 TIM5 CH2 neutral");
  check(outputs[1].writeMicroseconds(1000) && htim2.compare[0] == 1000, "lower tail PA15 independent channel");
  check(outputs[2].writeAngle(0) && htim1.compare[0] == 500 && htim5.compare[1] == 1500,
        "left fin PA8 does not overwrite upper tail PA1");
  check(outputs[3].writeAngle(180) && htim5.compare[0] == 2500, "right fin PA0 TIM5 CH1 endpoint");
  check(outputs[4].writeMicroseconds(500) && htim3.compare[2] == 1000, "ESC lower bound");
  check(outputs[5].writeMicroseconds(2500) && htim3.compare[3] == 2000, "ESC upper bound");
  ServoPwm invalid;
  check(!invalid.begin(kPa0, 0, PwmTimer::kTim5, 500, 2500), "logical channel zero rejected");
  check(!invalid.begin(kPa0, 5, PwmTimer::kTim5, 500, 2500), "logical channel five rejected");
  check(!invalid.writeAngle(90), "unstarted output cannot write");
  pwmResult = HAL_ERROR;
  check(!invalid.begin(kPa0, 1, PwmTimer::kTim5, 500, 2500), "HAL start error propagates");
  pwmResult = HAL_OK;

  huart1.Instance = USART1;
  LoRaUart uart;
  receiveImmediately = true;
  check(uart.begin() && huart1.RxState == HAL_UART_STATE_BUSY_RX, "continuous RX starts");
  uint8_t firstByte = 0;
  check(rxArms == 2 && uart.read(&firstByte, 1) == 1 && firstByte == 127,
        "immediate RX during initialization still rearms the interrupt");
  check(uart.txState() == UartTxState::Idle, "RX busy does not block TX");
  const char ack[] = "910,0,1,1,0,2,1\n";
  const auto sent = uart.tryWriteFrame(ack, sizeof(ack)-1);
  check(sent.bytes == sizeof(ack)-1 && !sent.error && txLength == sizeof(ack)-1 &&
        std::memcmp(txBytes, ack, txLength) == 0, "ACK submitted intact");
  check(uart.txState() == UartTxState::Busy && uart.tryWriteFrame(ack, sizeof(ack)-1).bytes == 0,
        "ongoing DMA is not overwritten");
  huart1.gState = HAL_UART_STATE_READY;
  check(uart.txState() == UartTxState::Idle, "TX drains while RX remains active");
  txResult = HAL_ERROR;
  check(uart.tryWriteFrame(ack, sizeof(ack)-1).error, "HAL TX error propagates");
  txResult = HAL_BUSY;
  const auto busy = uart.tryWriteFrame(ack, sizeof(ack)-1);
  check(busy.bytes == 0 && !busy.error, "HAL busy remains retryable");
  txResult = HAL_OK;
  *rxByte = 42; onUartRxComplete();
  uint8_t bytes[512]{};
  check(uart.read(bytes, sizeof(bytes)) == 1 && bytes[0] == 42, "IRQ callback feeds ring");
  for (unsigned i = 0; i < 600; ++i) uart.onRxByte(static_cast<uint8_t>(i));
  check(uart.read(bytes, sizeof(bytes)) == -1, "ring overflow invalidates the byte stream");
  for (unsigned i = 0; i < 300; ++i) uart.onRxByte(static_cast<uint8_t>(i));
  check(uart.read(bytes, 128) == 128 && uart.read(bytes, sizeof(bytes)) == 172 && bytes[171] == 43,
        "ring wraps across bounded reads");
  huart1.gState = HAL_UART_STATE_BUSY_TX;
  onUartError();
  check(receiveAborts == 1 && clearedReceiveErrors == 1 &&
      huart1.RxState == HAL_UART_STATE_BUSY_RX && huart1.gState == HAL_UART_STATE_BUSY_TX,
      "ORE/FE/NE recovery rearms RX without aborting TX DMA");
  check(uart.read(bytes, sizeof(bytes)) == -1, "UART error reports stream discontinuity");
  *rxByte=42; onUartRxComplete();
  check(uart.read(bytes, sizeof(bytes)) == 1 && bytes[0] == 42, "RX works after error recovery");
  rxResult = HAL_BUSY;
  onUartError();
  check(huart1.RxState == HAL_UART_STATE_READY, "failed rearm leaves RX restartable");
  rxResult = HAL_OK;
  check(uart.read(bytes, sizeof(bytes)) == -1 && huart1.RxState == HAL_UART_STATE_BUSY_RX,
      "RadioPoll retries a failed RX rearm");
  uart.stop();
  check(aborts == 1 && uart.txState() == UartTxState::Error && !uart.ready(), "stop aborts UART ownership");

  RgbStatusStrip rgb;
  check(rgb.begin() && waveformLength == 256, "eight pixels plus reset interval");
  check(!rgb.setColor(255, 0, 0), "in-flight waveform cannot be modified");
  HAL_TIM_PWM_PulseFinishedCallback(&htim4);
  check(rgb.setColor(255, 0, 0), "next waveform starts after completion");
  check(reinterpret_cast<uintptr_t>(waveform) % 4 == 0, "DMA buffer alignment");
  check(waveform[0] == 29 && waveform[10] == 58 && waveform[12] == 58,
        "GRB order and brightness encoded");
  bool resetLow = true;
  for (unsigned i = 192; i < 256; ++i) resetLow &= waveform[i] == 0;
  check(resetLow, "WS2812 reset tail remains low");
  std::printf("PASS: %u F405 BSP checks; HAL/electrical boundaries simulated\n", checks);
}
