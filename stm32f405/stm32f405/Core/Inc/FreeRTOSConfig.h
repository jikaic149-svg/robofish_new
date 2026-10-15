#pragma once
#include <stdint.h>

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t SystemCoreClock;
void Error_Handler(void);
void robofish_assert_failed(uint32_t line);
#ifdef __cplusplus
}
#endif

#define configUSE_PREEMPTION                     1
#define configCPU_CLOCK_HZ                       SystemCoreClock
#define configTICK_RATE_HZ                       1000U
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                     5
#define configMINIMAL_STACK_SIZE                 128U
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            0
#define configUSE_TIMERS                         0
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_TICKLESS_IDLE                  0
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         0
#define configKERNEL_PROVIDED_STATIC_MEMORY      1
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_NEWLIB_REENTRANT               0
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY          (15U << (8U - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (5U << (8U - configPRIO_BITS))
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark       1
#define configASSERT(condition) do { if (!(condition)) robofish_assert_failed(__LINE__); } while (0)
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
