#include "bsp/BoardInit.h"
#include "main.h"
#include "bsp/LoRaUart.h"

TIM_HandleTypeDef htim1{};
TIM_HandleTypeDef htim2{};
TIM_HandleTypeDef htim3{};
TIM_HandleTypeDef htim4{};
TIM_HandleTypeDef htim5{};
UART_HandleTypeDef huart1{};
DMA_HandleTypeDef hdma_tim4_ch3{};
DMA_HandleTypeDef hdma_usart1_tx{};

namespace {
void initPwmTimer(TIM_HandleTypeDef &timer, TIM_TypeDef *instance,
                  uint32_t prescaler, uint32_t period) {
  timer.Instance = instance;
  timer.Init.Prescaler = prescaler;
  timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer.Init.Period = period;
  timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&timer) != HAL_OK) Error_Handler();
}

void configurePwmChannel(TIM_HandleTypeDef &timer, uint32_t channel) {
  TIM_OC_InitTypeDef config{};
  config.OCMode = TIM_OCMODE_PWM1;
  config.Pulse = 0;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&timer, &config, channel) != HAL_OK) Error_Handler();
}
void SystemClock_Config() {
  RCC_OscInitTypeDef oscillator{};
  RCC_ClkInitTypeDef clocks{};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLM = 8;
  oscillator.PLL.PLLN = 336;
  oscillator.PLL.PLLP = RCC_PLLP_DIV2;
  oscillator.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) Error_Handler();
  clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV4;
  clocks.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void MX_GPIO_Init() {
  GPIO_InitTypeDef gpio{};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM5;
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Alternate = GPIO_AF1_TIM2;
  gpio.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Alternate = GPIO_AF1_TIM1;
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Alternate = GPIO_AF2_TIM3;
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Alternate = GPIO_AF2_TIM4;
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Alternate = GPIO_AF7_USART1;
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &gpio);

}

void MX_DMA_Init() {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_tim4_ch3.Instance = DMA1_Stream7;
  hdma_tim4_ch3.Init.Channel = DMA_CHANNEL_2;
  hdma_tim4_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim4_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim4_ch3.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim4_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim4_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim4_ch3.Init.Mode = DMA_NORMAL;
  hdma_tim4_ch3.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim4_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim4_ch3) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&htim4, hdma[TIM_DMA_ID_CC3], hdma_tim4_ch3);

  hdma_usart1_tx.Instance = DMA2_Stream7;
  hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void MX_TIM1_Init() { initPwmTimer(htim1, TIM1, 167, 19999); configurePwmChannel(htim1, TIM_CHANNEL_1); }
void MX_TIM2_Init() { initPwmTimer(htim2, TIM2, 83, 19999); configurePwmChannel(htim2, TIM_CHANNEL_1); }
void MX_TIM3_Init() { initPwmTimer(htim3, TIM3, 83, 19999); configurePwmChannel(htim3, TIM_CHANNEL_3); configurePwmChannel(htim3, TIM_CHANNEL_4); }
void MX_TIM4_Init() { initPwmTimer(htim4, TIM4, 0, 104); configurePwmChannel(htim4, TIM_CHANNEL_3); }
void MX_TIM5_Init() { initPwmTimer(htim5, TIM5, 83, 19999); configurePwmChannel(htim5, TIM_CHANNEL_1); configurePwmChannel(htim5, TIM_CHANNEL_2); }

void MX_USART1_UART_Init() {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

}  // namespace

extern "C" void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timer) {
  if (timer->Instance == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
  if (timer->Instance == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  if (timer->Instance == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  if (timer->Instance == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
  if (timer->Instance == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef *uart) {
  if (uart->Instance == USART1) __HAL_RCC_USART1_CLK_ENABLE();
}

extern "C" void USART1_IRQHandler() { HAL_UART_IRQHandler(&huart1); }
extern "C" void DMA1_Stream7_IRQHandler() { HAL_DMA_IRQHandler(&hdma_tim4_ch3); }
extern "C" void DMA2_Stream7_IRQHandler() { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
  if (uart->Instance == USART1) robofish::bsp::onUartRxComplete();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
  if (uart->Instance == USART1) robofish::bsp::onUartError();
}

namespace robofish::bsp {
void initializeBoard() {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();

}
}  // namespace robofish::bsp

namespace robofish::bsp {
uint32_t nowMs() { return HAL_GetTick(); }
}
