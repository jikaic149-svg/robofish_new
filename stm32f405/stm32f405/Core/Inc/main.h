#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_tim4_ch3;
extern DMA_HandleTypeDef hdma_usart1_tx;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif
