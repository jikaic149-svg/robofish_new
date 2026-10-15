#pragma once
#include "FreeRTOS.h"
struct StaticSemaphore_t { bool locked=false; };
using SemaphoreHandle_t=StaticSemaphore_t*;
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *s) {return s;}
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s,TickType_t) {if(s->locked)return pdFALSE;s->locked=true;return pdTRUE;}
inline void xSemaphoreGive(SemaphoreHandle_t s) {s->locked=false;}
