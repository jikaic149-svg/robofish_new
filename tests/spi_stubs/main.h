#pragma once
#include "../../stm32f405/stm32f405/tests/stubs/main.h"
#include "FreeRTOS.h"
#include <cassert>
constexpr uint32_t RESET=0,GPIO_PIN_12=1U<<12,GPIO_PIN_13=1U<<13,GPIO_PIN_14=1U<<14,GPIO_PIN_15=1U<<15;
constexpr uint32_t GPIO_PIN_SET=1,GPIO_MODE_AF_PP=2,GPIO_NOPULL=0,GPIO_SPEED_FREQ_VERY_HIGH=3,GPIO_AF5_SPI2=5;
struct GPIO_TypeDef {};
inline GPIO_TypeDef gpioB;
#define GPIOB (&gpioB)
struct GPIO_InitTypeDef {uint32_t Pin,Mode,Pull,Speed,Alternate;};
inline bool mockCsHigh=true,mockInsideDma=false,mockDelayedDisable=false;
inline unsigned mockStarts=0,mockResets=0;
inline bool mockFailStart=false;
inline uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef*,uint32_t) {return mockCsHigh?GPIO_PIN_SET:0;}
inline void HAL_GPIO_Init(GPIO_TypeDef*,GPIO_InitTypeDef*) {}
inline uint32_t HAL_GetTick() {return mockTick;}
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
constexpr uint32_t SPI_CR2_TXDMAEN=1,SPI_CR2_RXDMAEN=2,SPI_CR2_ERRIE=4,DMA_SxCR_EN=1,SPI_FLAG_OVR=0x40;
#define CLEAR_BIT(reg,bits) ((reg)&=~(bits))
#define SET_BIT(reg,bits) ((reg)|=(bits))
#define MODIFY_REG(reg,mask,value) ((reg)=((reg)&~(mask))|(value))
struct SPI_TypeDef {uint32_t CR2=0,SR=0;};
inline SPI_TypeDef spi2;
#define SPI2 (&spi2)
constexpr unsigned SPI_MODE_SLAVE=0,SPI_DIRECTION_2LINES=0,SPI_DATASIZE_8BIT=0,SPI_POLARITY_LOW=0,SPI_PHASE_1EDGE=0;
constexpr unsigned SPI_NSS_HARD_INPUT=0,SPI_FIRSTBIT_MSB=0,SPI_TIMODE_DISABLE=0,SPI_CRCCALCULATION_DISABLE=0;
constexpr unsigned DMA_CHANNEL_0=0,DMA_PERIPH_TO_MEMORY=0,DMA_MEMORY_TO_PERIPH=1,DMA_PINC_DISABLE=0,DMA_MINC_ENABLE=1;
constexpr unsigned DMA_PDATAALIGN_BYTE=0,DMA_MDATAALIGN_BYTE=0,DMA_NORMAL=0,DMA_PRIORITY_HIGH=0,DMA_FIFOMODE_DISABLE=0;
enum {HAL_DMA_STATE_READY,HAL_DMA_STATE_BUSY};
enum {HAL_SPI_STATE_RESET,HAL_SPI_STATE_READY,HAL_SPI_STATE_BUSY_TX_RX};
constexpr unsigned HAL_UNLOCKED=0,HAL_DMA_ERROR_NONE=0;
struct DMA_Stream_TypeDef {uint32_t CR=0,NDTR=0;};
inline DMA_Stream_TypeDef dmaRx,dmaTx;
#define DMA1_Stream3 (&dmaRx)
#define DMA1_Stream4 (&dmaTx)
struct DMA_InitTypeDef {unsigned Channel,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode;};
struct DMA_HandleTypeDef {DMA_Stream_TypeDef *Instance=nullptr;DMA_InitTypeDef Init{};unsigned State=HAL_DMA_STATE_READY,ErrorCode=0,Lock=0;bool complete=false;};
struct SPI_InitTypeDef {unsigned Mode,Direction,DataSize,CLKPolarity,CLKPhase,NSS,FirstBit,TIMode,CRCCalculation,CRCPolynomial;};
struct SPI_HandleTypeDef {SPI_TypeDef *Instance=nullptr;SPI_InitTypeDef Init{};unsigned State=HAL_SPI_STATE_RESET,Lock=0;DMA_HandleTypeDef *hdmarx=nullptr,*hdmatx=nullptr;};
inline SPI_HandleTypeDef *mockSpi=nullptr;
inline HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *s) {s->State=HAL_SPI_STATE_READY;mockSpi=s;return HAL_OK;}
inline HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *d) {d->State=HAL_DMA_STATE_READY;return HAL_OK;}
#define __HAL_LINKDMA(s,member,d) ((s)->member=&(d))
inline HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *s,uint8_t*,uint8_t*,uint16_t length) {
  assert(!mockInsideDma && mockCsHigh);
  assert(s->hdmarx->State==HAL_DMA_STATE_READY && s->hdmatx->State==HAL_DMA_STATE_READY);
  ++mockStarts;s->State=HAL_SPI_STATE_BUSY_TX_RX;
  s->Instance->CR2=SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN|SPI_CR2_ERRIE;
  s->hdmarx->State=HAL_DMA_STATE_BUSY;s->hdmarx->Instance->CR=DMA_SxCR_EN;s->hdmarx->Instance->NDTR=length;
  if(mockFailStart) {mockFailStart=false;return HAL_ERROR;}
  s->hdmatx->State=HAL_DMA_STATE_BUSY;s->hdmatx->Instance->CR=DMA_SxCR_EN;s->hdmatx->Instance->NDTR=length;
  return HAL_OK;
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef*);
inline void HAL_DMA_IRQHandler(DMA_HandleTypeDef *d) {
  mockInsideDma=true;
  if(d->complete){d->complete=false;d->State=HAL_DMA_STATE_READY;d->Instance->CR=0;
    if(d==mockSpi->hdmarx){mockSpi->State=HAL_SPI_STATE_READY;mockSpi->Instance->CR2=0;HAL_SPI_TxRxCpltCallback(mockSpi);}}
  mockInsideDma=false;
}
inline void HAL_SPI_IRQHandler(SPI_HandleTypeDef*) {}
inline void mockDisable(DMA_HandleTypeDef *d) {if(!mockDelayedDisable)d->Instance->CR=0;}
#define __HAL_DMA_DISABLE(d) mockDisable(d)
#define __HAL_SPI_DISABLE(s) ((void)(s))
#define __HAL_DMA_GET_COUNTER(d) ((d)->Instance->NDTR)
#define __HAL_SPI_GET_FLAG(s,f) ((s)->Instance->SR&(f))
#define __HAL_DMA_GET_TC_FLAG_INDEX(d) 1
#define __HAL_DMA_GET_HT_FLAG_INDEX(d) 2
#define __HAL_DMA_GET_TE_FLAG_INDEX(d) 4
#define __HAL_DMA_GET_DME_FLAG_INDEX(d) 8
#define __HAL_DMA_GET_FE_FLAG_INDEX(d) 16
#define __HAL_DMA_CLEAR_FLAG(d,f) ((void)(f))
#define __HAL_RCC_SPI2_FORCE_RESET() (++mockResets,spi2=SPI_TypeDef{})
#define __HAL_RCC_SPI2_RELEASE_RESET() ((void)0)
enum IRQn_Type {DMA1_Stream3_IRQn,DMA1_Stream4_IRQn,SPI2_IRQn,EXTI15_10_IRQn};
inline void HAL_NVIC_SetPriority(IRQn_Type,unsigned,unsigned) {}
inline void HAL_NVIC_EnableIRQ(IRQn_Type) {}
inline void HAL_NVIC_ClearPendingIRQ(IRQn_Type) {}
struct Syscfg {uint32_t EXTICR[4]{};};
inline Syscfg syscfg;
#define SYSCFG (&syscfg)
struct Exti {uint32_t RTSR=0,FTSR=0,IMR=0,pending=0;};
inline Exti exti;
#define EXTI (&exti)
constexpr unsigned SYSCFG_EXTICR4_EXTI12=0xf,SYSCFG_EXTICR4_EXTI12_PB=1;
#define __HAL_GPIO_EXTI_GET_IT(pin) (exti.pending&(pin))
#define __HAL_GPIO_EXTI_CLEAR_IT(pin) (exti.pending&=~(pin))
