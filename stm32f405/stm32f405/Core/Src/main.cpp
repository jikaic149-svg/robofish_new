#include "main.h"
#include "bsp/RuntimeSafety.h"
#include "bsp/BoardInit.h"
#include "task/RobotTasks.h"
#include <FreeRTOS.h>
#include <task.h>

int main() {
  robofish::bsp::initializeSafety();
  robofish::bsp::initializeBoard();
  if (!robofish::task::startRobotTasks()) Error_Handler();
  vTaskStartScheduler();
  Error_Handler();
}

extern "C" void xPortSysTickHandler();
extern "C" void SysTick_Handler() {
  HAL_IncTick();
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) xPortSysTickHandler();
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *) {
  robofish::bsp::fail(robofish::bsp::FaultReason::StackOverflow);
}

extern "C" void Error_Handler() {
  robofish::bsp::fail(robofish::bsp::FaultReason::Init);
}

extern "C" void robofish_assert_failed(uint32_t line) {
  robofish::bsp::fail(robofish::bsp::FaultReason::Assert, line);
}
