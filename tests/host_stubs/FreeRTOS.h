#pragma once
#include <cstdint>
using BaseType_t=int;
using UBaseType_t=unsigned;
using TickType_t=uint32_t;
using StackType_t=uint32_t;
struct StaticTask_t {};
using TaskHandle_t=StaticTask_t*;
using TaskFunction_t=void(*)(void*);
inline TickType_t mockTick=0;
constexpr BaseType_t pdTRUE=1,pdFALSE=0;
#define pdMS_TO_TICKS(ms) (ms)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define portYIELD_FROM_ISR(wake) ((void)(wake))
