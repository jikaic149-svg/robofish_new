#include "task/RobotTasks.h"
#include "main.h"
#include "bsp/BatteryAdc.h"
#include "bsp/SpiLink.h"
#include "bsp/RgbStatusStrip.h"
#include "config/ControlConfig.h"
#include "device/motion/RoboFishDriver.h"
#include "service/lora/LoRaService.h"
#include "task/MotionController.h"
#include <FreeRTOS.h>
#include <task.h>

namespace {
RoboFishDriver gDriver;
MotionController gMotion(gDriver);
RgbStatusStrip gStatusStrip;

bool submitLoRa(const robofish::lora::Command &command, void *) {
  if (command.code == 8 || command.code == 18 || command.code == 19) return false;
  return gMotion.submitLoRa(command);
}

robofish::lora::LoRaService gLoRa(submitLoRa, nullptr);
StaticTask_t gRadioTaskStorage{};
StackType_t gRadioTaskStack[512]{};

void radioTask(void *) {
  // Device locks and HAL/DMA callbacks run with the scheduler already active.
  if (!robofish::bsp::startSpiLink(gMotion)) Error_Handler();
  if (!gDriver.begin()) Error_Handler();
  gStatusStrip.begin();
  vTaskDelay(pdMS_TO_TICKS(1));
  gStatusStrip.setColor(32, 16, 0);
  vTaskDelay(pdMS_TO_TICKS(robofish::config::lora::kEscArmMs));
  if (!gMotion.begin() || !gLoRa.begin() ||
      !robofish::bsp::startBatteryAdc(gMotion)) Error_Handler();
  gStatusStrip.setColor(0, 24, 0);
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    robofish::bsp::pollBatteryAdc();
    gLoRa.poll(HAL_GetTick());
    gMotion.observeLoRa(gLoRa.status());
    gMotion.update();
    gMotion.noteRadioCycle(HAL_GetTick());
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(robofish::config::radio::kPollPeriodMs));
  }
}

}  // namespace

namespace robofish::task {
bool startRobotTasks() {
  return xTaskCreateStatic(radioTask, "RadioPoll",
                         sizeof(gRadioTaskStack) / sizeof(StackType_t), nullptr,
                         2, gRadioTaskStack, &gRadioTaskStorage) != nullptr;
}
}  // namespace robofish::task
