#include "bsp/SpiLink.h"
#include "main.h"
#include "config/ControlConfig.h"
#include "spi_link_protocol.h"
#include "task/MotionController.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <atomic>
#include <cstring>

namespace {
SPI_HandleTypeDef spi{};
DMA_HandleTypeDef receiveDma{}, transmitDma{};
MotionController *controller=nullptr;
StaticQueue_t receiveQueueStorage{};
uint8_t receiveQueueBytes[4 * RF_SPI_FRAME_BYTES]{};
QueueHandle_t receiveQueue=nullptr;
StaticTask_t taskStorage{};
StackType_t taskStack[512]{};
alignas(4) uint8_t receiveBytes[RF_SPI_FRAME_BYTES]{};
alignas(4) uint8_t transmitBytes[RF_SPI_FRAME_BYTES]{};
alignas(4) uint8_t publishedBytes[RF_SPI_FRAME_BYTES]{};
std::atomic_uint32_t dropped{0}, transferErrors{0};
std::atomic_bool running{false};
// Hardware state is serialized by equal-priority IRQs or a task critical section.
enum class TransferPhase : uint8_t { Idle, Active, Completed, Recovering };
TransferPhase phase = TransferPhase::Idle;

void recoverTransfer() {
    phase = TransferPhase::Recovering;
    CLEAR_BIT(spi.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE);
    __HAL_SPI_DISABLE(&spi);
    __HAL_DMA_DISABLE(&receiveDma);
    __HAL_DMA_DISABLE(&transmitDma);
}
void clearDma(DMA_HandleTypeDef &dma) {
    __HAL_DMA_CLEAR_FLAG(&dma, __HAL_DMA_GET_TC_FLAG_INDEX(&dma) |
        __HAL_DMA_GET_HT_FLAG_INDEX(&dma) | __HAL_DMA_GET_TE_FLAG_INDEX(&dma) |
        __HAL_DMA_GET_DME_FLAG_INDEX(&dma) | __HAL_DMA_GET_FE_FLAG_INDEX(&dma));
    dma.State = HAL_DMA_STATE_READY;
    dma.ErrorCode = HAL_DMA_ERROR_NONE;
    dma.Lock = HAL_UNLOCKED;
}
BaseType_t progressTransfer(bool fromIsr) {
    BaseType_t wake = pdFALSE;
    if (!running || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_SET) return wake;
    if (phase == TransferPhase::Recovering) {
        // EN clears asynchronously. Return to the task/IRQ instead of spinning.
        if ((receiveDma.Instance->CR | transmitDma.Instance->CR) & DMA_SxCR_EN) return wake;
        clearDma(receiveDma); clearDma(transmitDma);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);
        __HAL_RCC_SPI2_FORCE_RESET(); __HAL_RCC_SPI2_RELEASE_RESET();
        spi.State = HAL_SPI_STATE_RESET; spi.Lock = HAL_UNLOCKED;
        if (HAL_SPI_Init(&spi) != HAL_OK) return wake;
        phase = TransferPhase::Idle;
    }
    if (phase == TransferPhase::Active) {
        const uint32_t remaining = __HAL_DMA_GET_COUNTER(&receiveDma);
        if (remaining != RF_SPI_FRAME_BYTES && remaining != 0) {
            transferErrors.fetch_add(1, std::memory_order_relaxed);
            recoverTransfer();  // CS rose before the entire fixed frame arrived.
        }
        return wake;
    }
    if (receiveDma.State != HAL_DMA_STATE_READY || transmitDma.State != HAL_DMA_STATE_READY ||
        spi.State != HAL_SPI_STATE_READY) return wake;
    if (phase == TransferPhase::Completed) {
        if (__HAL_DMA_GET_COUNTER(&receiveDma) != 0 || __HAL_DMA_GET_COUNTER(&transmitDma) != 0 ||
            __HAL_SPI_GET_FLAG(&spi, SPI_FLAG_OVR) != RESET) {
            transferErrors.fetch_add(1, std::memory_order_relaxed);
            recoverTransfer();
            return wake;
        }
        const BaseType_t sent = fromIsr ? xQueueSendFromISR(receiveQueue, receiveBytes, &wake)
                                       : xQueueSend(receiveQueue, receiveBytes, 0);
        if (sent != pdTRUE) dropped.fetch_add(1, std::memory_order_relaxed);
        phase = TransferPhase::Idle;
    }
    // Buffers change only after CS release and BOTH DMA IRQ handlers have returned.
    __HAL_SPI_DISABLE(&spi);
    std::memcpy(transmitBytes, publishedBytes, sizeof(transmitBytes));
    phase = TransferPhase::Active;
    if (HAL_SPI_TransmitReceive_DMA(&spi, transmitBytes, receiveBytes, RF_SPI_FRAME_BYTES) != HAL_OK) {
        transferErrors.fetch_add(1, std::memory_order_relaxed);
        recoverTransfer();  // Also unwinds a partially started RX DMA.
    }
    return wake;
}
void linkTask(void *) {
    uint32_t session=0, lastSequence=0, responseSequence=0, lastCommand=0;
    uint32_t received=0, invalid=0, lastReceiveMs=0, lastSnapshot=0;
    uint8_t commandResult=0;
    bool online=false, ready=false;
    MotionStatus status{};
    MotionDiagnostics diagnostics{};
    uint8_t packet[RF_SPI_FRAME_BYTES]{}, response[RF_SPI_FRAME_BYTES]{};
    for (;;) {
        taskENTER_CRITICAL();
        (void)progressTransfer(false);  // Bounded recovery even if no more IRQ arrives.
        taskEXIT_CRITICAL();
        if (xQueueReceive(receiveQueue,packet,pdMS_TO_TICKS(RF_SPI_PERIOD_MS))==pdTRUE) {
            if (!rf_spi_valid(packet,sizeof(packet),RF_SPI_REQUEST) || !rf_spi_u32(packet+8))
                ++invalid;
            else {
                uint32_t incomingSession=rf_spi_u32(packet+8);
                if (session!=incomingSession) {
                    session=incomingSession; lastSequence=0; lastCommand=0; commandResult=0;
                }
                uint32_t sequence=rf_spi_u32(packet+12);
                if (rf_spi_after(sequence,lastSequence)) {
                    lastSequence=sequence; lastReceiveMs=HAL_GetTick(); ++received;
                    uint32_t commandId=rf_spi_u32(packet+24);
                    if (commandId && rf_spi_after(commandId,lastCommand)) {
                        lastCommand=commandId;
                        // Reserved command frames are acknowledged as rejected until
                        // the LoRa heartbeat/takeover contract is defined.
                        commandResult=RF_SPI_RESULT_REJECTED;
                    }
                }
            }
        }
        uint32_t now=HAL_GetTick();
        online=received && (uint32_t)(now-lastReceiveMs)<RF_SPI_OFFLINE_MS;
        controller->publishSpiLink(lastReceiveMs, received != 0);
        if (now-lastSnapshot>=20U || !ready) {
            lastSnapshot=now;
            ready=controller->getStatus(status);
            if (ready) (void)controller->getDiagnostics(diagnostics);
        }
        /* Link expiry is diagnostic only; it never changes LoRa or motion. */
        if (++responseSequence==0) responseSequence=1;
        rf_spi_begin(response,RF_SPI_RESPONSE,session,responseSequence,lastSequence,now*1000U);
        if (ready) response[5]|=RF_SPI_READY;
        rf_spi_put32(response+24,lastCommand); response[28]=commandResult;
        response[29]=static_cast<uint8_t>(status.mode);
        response[30]=static_cast<uint8_t>(status.action);
        response[31]=status.powerPercent; response[32]=status.turnStrengthPercent;
        response[33]=status.failsafeActive; response[34]=static_cast<uint8_t>(diagnostics.selectedSource);
        response[35]=static_cast<uint8_t>(robofish::batteryCondition(diagnostics.battery, now,
            robofish::config::battery::kMaximumSampleAgeMs));
        rf_spi_put32(response+36,diagnostics.frameAttempts);
        rf_spi_put32(response+40,diagnostics.frameFailures);
        rf_spi_put32(response+44,received); rf_spi_put32(response+48,invalid);
        rf_spi_put32(response+52,dropped.load(std::memory_order_relaxed));
        rf_spi_put32(response+56,transferErrors.load(std::memory_order_relaxed));
        response[60]=online;
        response[61]=diagnostics.battery.valid &&
            diagnostics.batteryAgeMs <= robofish::config::battery::kMaximumSampleAgeMs;
        rf_spi_put16(response+62,static_cast<uint16_t>(
            diagnostics.batteryAgeMs > 65535U ? 65535U : diagnostics.batteryAgeMs));
        rf_spi_put16(response+64,static_cast<uint16_t>(diagnostics.lastWrittenFrame.tailFrontDegrees*100.0f));
        rf_spi_put16(response+66,static_cast<uint16_t>(diagnostics.lastWrittenFrame.tailRearDegrees*100.0f));
        rf_spi_put16(response+68,static_cast<uint16_t>(diagnostics.lastWrittenFrame.pectoralLeftDegrees*100.0f));
        rf_spi_put16(response+70,static_cast<uint16_t>(diagnostics.lastWrittenFrame.pectoralRightDegrees*100.0f));
        rf_spi_put16(response+72,static_cast<uint16_t>(diagnostics.lastWrittenFrame.leftPropulsionPercent));
        rf_spi_put16(response+74,static_cast<uint16_t>(diagnostics.lastWrittenFrame.rightPropulsionPercent));
        rf_spi_put16(response+76,static_cast<uint16_t>(status.pitchAngle));
        rf_spi_put16(response+78,diagnostics.battery.millivolts);
        rf_spi_seal(response);
        taskENTER_CRITICAL();
        std::memcpy(publishedBytes,response,sizeof(response));
        taskEXIT_CRITICAL();
    }
}
bool initializeHardware() {
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin=GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode=GPIO_MODE_AF_PP; gpio.Pull=GPIO_NOPULL;
    gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate=GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB,&gpio);
    spi.Instance=SPI2; spi.Init.Mode=SPI_MODE_SLAVE;
    spi.Init.Direction=SPI_DIRECTION_2LINES; spi.Init.DataSize=SPI_DATASIZE_8BIT;
    spi.Init.CLKPolarity=SPI_POLARITY_LOW; spi.Init.CLKPhase=SPI_PHASE_1EDGE;
    spi.Init.NSS=SPI_NSS_HARD_INPUT; spi.Init.FirstBit=SPI_FIRSTBIT_MSB;
    spi.Init.TIMode=SPI_TIMODE_DISABLE; spi.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial=7;
    if (HAL_SPI_Init(&spi)!=HAL_OK) return false;
    receiveDma.Instance=DMA1_Stream3;
    receiveDma.Init.Channel=DMA_CHANNEL_0; receiveDma.Init.Direction=DMA_PERIPH_TO_MEMORY;
    receiveDma.Init.PeriphInc=DMA_PINC_DISABLE; receiveDma.Init.MemInc=DMA_MINC_ENABLE;
    receiveDma.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
    receiveDma.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
    receiveDma.Init.Mode=DMA_NORMAL; receiveDma.Init.Priority=DMA_PRIORITY_HIGH;
    receiveDma.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&receiveDma)!=HAL_OK) return false;
    transmitDma.Instance=DMA1_Stream4; transmitDma.Init=receiveDma.Init;
    transmitDma.Init.Direction=DMA_MEMORY_TO_PERIPH;
    if (HAL_DMA_Init(&transmitDma)!=HAL_OK) return false;
    __HAL_LINKDMA(&spi,hdmarx,receiveDma); __HAL_LINKDMA(&spi,hdmatx,transmitDma);
    const IRQn_Type interrupts[]={DMA1_Stream3_IRQn,DMA1_Stream4_IRQn,SPI2_IRQn,EXTI15_10_IRQn};
    for (IRQn_Type irq : interrupts) {
        HAL_NVIC_SetPriority(irq,5,0); HAL_NVIC_EnableIRQ(irq);
    }
    /* Keep PB12 in AF5; EXTI observes NSS rising edges without changing its GPIO mode. */
    MODIFY_REG(SYSCFG->EXTICR[3],SYSCFG_EXTICR4_EXTI12,SYSCFG_EXTICR4_EXTI12_PB);
    SET_BIT(EXTI->RTSR,GPIO_PIN_12); CLEAR_BIT(EXTI->FTSR,GPIO_PIN_12);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12); SET_BIT(EXTI->IMR,GPIO_PIN_12);
    return true;
}
}
namespace robofish::bsp {
bool startSpiLink(MotionController &motion) {
    controller=&motion;
    receiveQueue=xQueueCreateStatic(4,RF_SPI_FRAME_BYTES,receiveQueueBytes,&receiveQueueStorage);
    rf_spi_begin(publishedBytes,RF_SPI_RESPONSE,0,0,0,HAL_GetTick()*1000U);
    rf_spi_seal(publishedBytes);
    if (!receiveQueue || !initializeHardware()) return false;
    if (!xTaskCreateStatic(linkTask,"SpiLink",512,nullptr,3,taskStack,&taskStorage)) return false;
    running=true;
    taskENTER_CRITICAL();
    (void)progressTransfer(false);
    taskEXIT_CRITICAL();
    return true; // If the master currently owns CS, arm on its next rising edge.
}
}
extern "C" void DMA1_Stream3_IRQHandler() {
    HAL_DMA_IRQHandler(&receiveDma);
    portYIELD_FROM_ISR(progressTransfer(true));
}
extern "C" void DMA1_Stream4_IRQHandler() {
    HAL_DMA_IRQHandler(&transmitDma);
    portYIELD_FROM_ISR(progressTransfer(true));
}
extern "C" void SPI2_IRQHandler() {
    HAL_SPI_IRQHandler(&spi);
    portYIELD_FROM_ISR(progressTransfer(true));
}
extern "C" void EXTI15_10_IRQHandler() {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
        portYIELD_FROM_ISR(progressTransfer(true));
    }
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle) {
    if (handle == &spi && phase == TransferPhase::Active)
        phase = TransferPhase::Completed; // Do not rearm inside the RX callback.
}
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle) {
    if (handle == &spi) {
        transferErrors.fetch_add(1, std::memory_order_relaxed);
        recoverTransfer();
    }
}
