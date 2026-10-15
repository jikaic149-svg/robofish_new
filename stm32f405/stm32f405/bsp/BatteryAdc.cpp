#include "bsp/BatteryAdc.h"
#include <atomic>
#include "main.h"
#include "stm32f4xx_ll_adc.h"
#include "algorithm/BatteryVoltage.h"
#include "bsp/BoardResources.h"
#include "config/ControlConfig.h"
#include "task/MotionController.h"
namespace {
ADC_HandleTypeDef adc{};
DMA_HandleTypeDef dma{};
alignas(4) uint16_t dmaValues[2]{};
struct RawSample { uint16_t battery, reference; uint32_t sampledAtMs; bool valid; };
StaticQueue_t rawQueueStorage{};
alignas(4) uint8_t rawQueueBytes[sizeof(RawSample)]{};
QueueHandle_t rawQueue = nullptr;
MotionController *consumer = nullptr;
std::atomic_bool busy{false};
uint32_t startedAt = 0, lastStart = 0, sequence = 0;
bool attempted = false, filtered = false;
uint16_t filteredMv = 0;
void publish(const RawSample &raw) {
  const auto value = raw.valid ? robofish::algorithm::batteryVoltage(raw.battery,
      raw.reference, *VREFINT_CAL_ADDR, VREFINT_CAL_VREF,
      robofish::bsp::kBatteryDividerUpperOhms, robofish::bsp::kBatteryDividerLowerOhms)
      : robofish::algorithm::BatteryVoltage{};
  robofish::BatterySample sample;
  sample.sequence = ++sequence;
  sample.sampledAtMs = raw.sampledAtMs;
  sample.adcCounts = raw.battery;
  sample.vddaMillivolts = value.vddaMillivolts;
  sample.valid = value.valid;
  sample.rawMillivolts = sample.millivolts = value.millivolts;
  // Classify before filtering: a previously healthy display must not mask a fault.
  if (robofish::batteryCondition(sample, raw.sampledAtMs,
      robofish::config::battery::kMaximumSampleAgeMs) == robofish::BatteryCondition::Normal) {
    if (!filtered) { filteredMv = value.millivolts; filtered = true; }
    else filteredMv = uint16_t(int32_t(filteredMv) +
        (int32_t(value.millivolts) - filteredMv) / int32_t(robofish::config::battery::kFilterDivisor));
    sample.millivolts = filteredMv;
  } else filtered = false;
  (void)consumer->publishBatterySample(sample);
}
void publishFromIsr(bool valid) {
  RawSample sample{dmaValues[0], dmaValues[1], HAL_GetTick(), valid};
  BaseType_t wake = pdFALSE;
  xQueueOverwriteFromISR(rawQueue, &sample, &wake);
  busy.store(false, std::memory_order_release);
  portYIELD_FROM_ISR(wake);
}
}
namespace robofish::bsp {
bool startBatteryAdc(MotionController &controller) {
  consumer = &controller;
  rawQueue = xQueueCreateStatic(1, sizeof(RawSample), rawQueueBytes, &rawQueueStorage);
  if (!rawQueue) return false;
  __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_ADC1_CLK_ENABLE(); __HAL_RCC_DMA2_CLK_ENABLE();
  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_2; gpio.Mode = GPIO_MODE_ANALOG; gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);
  dma.Instance = DMA2_Stream0; dma.Init.Channel = DMA_CHANNEL_0;
  dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dma.Init.PeriphInc = DMA_PINC_DISABLE; dma.Init.MemInc = DMA_MINC_ENABLE;
  dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  dma.Init.Mode = DMA_NORMAL; dma.Init.Priority = DMA_PRIORITY_LOW;
  dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&dma) != HAL_OK) return false;
  adc.Instance = ADC1;
  adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  adc.Init.Resolution = ADC_RESOLUTION_12B; adc.Init.ScanConvMode = ENABLE;
  adc.Init.ContinuousConvMode = DISABLE; adc.Init.DiscontinuousConvMode = DISABLE;
  adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc.Init.DataAlign = ADC_DATAALIGN_RIGHT; adc.Init.NbrOfConversion = 2;
  adc.Init.DMAContinuousRequests = DISABLE; adc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  __HAL_LINKDMA(&adc, DMA_Handle, dma);
  if (HAL_ADC_Init(&adc) != HAL_OK) return false;
  ADC_ChannelConfTypeDef channel{};
  channel.Channel = ADC_CHANNEL_12; channel.Rank = 1;
  channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&adc, &channel) != HAL_OK) return false;
  channel.Channel = ADC_CHANNEL_VREFINT; channel.Rank = 2;
  if (HAL_ADC_ConfigChannel(&adc, &channel) != HAL_OK) return false;
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(ADC_IRQn, 5, 0); HAL_NVIC_EnableIRQ(ADC_IRQn);
  return true;
}
void pollBatteryAdc() {
  RawSample sample{};
  if (xQueueReceive(rawQueue, &sample, 0) == pdTRUE) {
    if (!sample.valid) (void)HAL_ADC_Stop_DMA(&adc);
    publish(sample);
  }
  const uint32_t now = HAL_GetTick();
  if (busy.load(std::memory_order_acquire)) {
    if (now - startedAt < config::battery::kConversionTimeoutMs) return;
    (void)HAL_ADC_Stop_DMA(&adc);
    busy.store(false, std::memory_order_release);
    publish({0, 0, now, false});
  }
  if (attempted && now - lastStart < config::battery::kSamplePeriodMs) return;
  attempted = true; startedAt = lastStart = now;
  busy.store(true, std::memory_order_release);
  if (HAL_ADC_Start_DMA(&adc, reinterpret_cast<uint32_t *>(dmaValues), 2) != HAL_OK) {
    (void)HAL_ADC_Stop_DMA(&adc);
    busy.store(false, std::memory_order_release);
    publish({0, 0, now, false});
  }
}
}
extern "C" void DMA2_Stream0_IRQHandler() { HAL_DMA_IRQHandler(&dma); }
extern "C" void ADC_IRQHandler() { HAL_ADC_IRQHandler(&adc); }
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle) {
  if (handle == &adc) publishFromIsr(true);
}
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle) {
  if (handle == &adc) publishFromIsr(false);
}
