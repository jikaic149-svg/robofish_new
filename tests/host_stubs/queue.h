#pragma once
#include "FreeRTOS.h"
#include <cstring>
struct StaticQueue_t { uint8_t *bytes=nullptr;unsigned size=0,capacity=0,read=0,write=0,count=0; };
using QueueHandle_t=StaticQueue_t*;
inline QueueHandle_t xQueueCreateStatic(unsigned capacity,unsigned size,uint8_t *bytes,StaticQueue_t *q) {*q={bytes,size,capacity,0,0,0};return q;}
inline BaseType_t xQueueSend(QueueHandle_t q,const void *value,TickType_t) {if(q->count==q->capacity)return pdFALSE;std::memcpy(q->bytes+q->write*q->size,value,q->size);q->write=(q->write+1)%q->capacity;++q->count;return pdTRUE;}
inline BaseType_t xQueueReceive(QueueHandle_t q,void *value,TickType_t) {if(!q->count)return pdFALSE;std::memcpy(value,q->bytes+q->read*q->size,q->size);q->read=(q->read+1)%q->capacity;--q->count;return pdTRUE;}
inline BaseType_t xQueueOverwrite(QueueHandle_t q,const void *value) {q->read=q->write=q->count=0;return xQueueSend(q,value,0);}
inline BaseType_t xQueueSendFromISR(QueueHandle_t q,const void *value,BaseType_t*) {return xQueueSend(q,value,0);}
