#pragma once
#include "FreeRTOS.h"
inline TaskHandle_t xTaskCreateStatic(TaskFunction_t,const char*,uint32_t,void*,UBaseType_t,StackType_t*,StaticTask_t *task) { return task; }
inline TickType_t xTaskGetTickCount() { return mockTick; }
inline void vTaskDelayUntil(TickType_t *wake,TickType_t period) { mockTick=*wake+=period; }
