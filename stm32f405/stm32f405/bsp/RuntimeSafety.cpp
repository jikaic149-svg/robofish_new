#include "bsp/RuntimeSafety.h"
#include "main.h"
#include "stm32_boot.h"
#include "boot/BootStore.h"
#include "config/RuntimeConfig.h"
#include "config/ControlConfig.h"
#include <atomic>

namespace {
enum class FaultStage { Normal, ControlledNeutral, Fatal };
std::atomic<FaultStage> fault{FaultStage::Normal};
uint32_t controlledStartedMs = 0;
uint32_t previousFault = 0, previousReset = 0;
bool watchdogArmed = false;
std::atomic_bool confirmed{false};
bool trial = false, confirmationAttempted = false;
unsigned healthyStartupCycles = 0;
uint32_t applicationSlot = RF_BOOT_NONE;
enum class WatchdogPhase { Startup, Updating, Running };
WatchdogPhase watchdogPhase = WatchdogPhase::Startup; // Only MotionControl writes.
__attribute__((section(".noinit.fault"), used, aligned(8))) volatile rf_fault_record_t faultRecord;

void record(robofish::bsp::FaultReason reason, uint32_t detail, uint32_t pc, uint32_t lr) {
  faultRecord.commit = 0;
  faultRecord.magic = RF_FAULT_MAGIC; faultRecord.version = 1;
  faultRecord.reason = static_cast<uint32_t>(reason);
  faultRecord.pc = pc; faultRecord.lr = lr; faultRecord.detail = detail;
  faultRecord.reset_flags = RCC->CSR; faultRecord.sequence = 0;
  faultRecord.tick = watchdogArmed ? HAL_GetTick() : 0;
  for (auto &value : faultRecord.reserved) value = 0;
  faultRecord.crc = rf_crc_bytes(const_cast<const rf_fault_record_t *>(&faultRecord),
                               offsetof(rf_fault_record_t, crc));
  __DMB(); faultRecord.commit = RF_FAULT_COMMIT; __DSB();
}
}

// Called BEFORE SystemInit, data/BSS and constructors. No globals, HAL or waits.
extern "C" void robofish_early_watchdog_start() {
  IWDG->KR = 0xCCCCU;
  IWDG->KR = 0x5555U;
  IWDG->PR = 4U;      // /64 * 4096: 8.192s at nominal LSI; verify real tolerance.
  IWDG->RLR = 4095U;  // Same as reset RLR: early reload is safe before update finishes.
  IWDG->KR = 0xAAAAU;
}

namespace robofish::bsp {
bool faultLatched() { return fault.load(std::memory_order_acquire) != FaultStage::Normal; }
bool controlledExitActive() { return fault.load(std::memory_order_acquire) == FaultStage::ControlledNeutral; }
void initializeSafety() {
  watchdogArmed = true; // The generated Reset_Handler has already enabled IWDG.
  const uint32_t vector = SCB->VTOR;
  if (vector == RF_APP_A || vector == RF_APP_B) {
    applicationSlot = vector == RF_APP_B ? 1U : 0U;
    rf_boot_record_t boot;
    if (rf_boot_latest(&boot, nullptr)) {
      // A verified older confirmed slot may be selected even if persisting the
      // fallback selection failed. Candidate slots never carry this bit.
      confirmed = (boot.confirmed_mask & (1U << applicationSlot)) != 0;
      trial = boot.state == RF_BOOT_TRYING && boot.candidate == applicationSlot;
    }
  }
  previousReset = RCC->CSR;
  if (!(previousReset & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) &&
      faultRecord.commit == RF_FAULT_COMMIT && faultRecord.magic == RF_FAULT_MAGIC &&
      faultRecord.version == 1 && faultRecord.crc ==
      rf_crc_bytes(const_cast<const rf_fault_record_t *>(&faultRecord), offsetof(rf_fault_record_t, crc)))
    previousFault = faultRecord.reason;
  faultRecord.commit = 0; // Consume once, never attribute an old crash to a later reset.
  RCC->CSR |= RCC_CSR_RMVF;
}
void serviceWatchdog(bool healthy, bool neutralSelfTest) {
  if (fault.load(std::memory_order_acquire) == FaultStage::Fatal) return;
  if (controlledExitActive() && uint32_t(HAL_GetTick() - controlledStartedMs) >= config::safety::kControlledNeutralMs) {
    fault.store(FaultStage::Fatal, std::memory_order_release);
    NVIC_SystemReset();
    return;
  }
  if (!healthy) { healthyStartupCycles = 0; return; }
  if (!confirmed) {
    if (!neutralSelfTest) { healthyStartupCycles = 0; return; }
    if (!trial || confirmationAttempted || ++healthyStartupCycles < 3) return;
    confirmationAttempted = true;
    rf_boot_record_t boot;
    if (rf_boot_latest(&boot, nullptr) && boot.state == RF_BOOT_TRYING &&
        boot.candidate == applicationSlot) {
      ++boot.sequence;
      boot.confirmed = applicationSlot; boot.confirmed_mask |= 1U << applicationSlot;
      boot.candidate = RF_BOOT_NONE; boot.state = RF_BOOT_STABLE;
      confirmed = rf_boot_append(&boot) != 0;
    }
    // Keep startup budget during the flash transaction; recheck all tasks next cycle.
    return;
  }
  if (watchdogPhase == WatchdogPhase::Startup) {
    IWDG->KR = 0x5555U;
    IWDG->PR = 4U; IWDG->RLR = 255U; // /64, 512ms nominal; verify LSI extremes on board.
    watchdogPhase = WatchdogPhase::Updating;
  }
  if (IWDG->SR != 0U) return; // No polling or blocking in the control loop.
  IWDG->KR = 0xAAAAU;
  watchdogPhase = WatchdogPhase::Running;
}
void emergencyNeutral() {
  // Known 1MHz PWM timers; write directly without HAL/RTOS locks. No PWM disable.
  if (RCC->APB1ENR & RCC_APB1ENR_TIM3EN) {
    TIM3->CCR3 = 1500U; TIM3->CCR4 = 1500U; TIM3->EGR = TIM_EGR_UG;
  }
  const auto pulse = [](int degrees) { return 500U + static_cast<uint32_t>(degrees) * 2000U / 180U; };
  if (RCC->APB1ENR & RCC_APB1ENR_TIM5EN) {
    TIM5->CCR1 = pulse(config::lora::kPectoralRightCenter);
    TIM5->CCR2 = pulse(config::lora::kTailFrontCenter); TIM5->EGR = TIM_EGR_UG;
  }
  if (RCC->APB1ENR & RCC_APB1ENR_TIM2EN) {
    TIM2->CCR1 = pulse(config::lora::kTailRearCenter); TIM2->EGR = TIM_EGR_UG;
  }
  if (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) {
    TIM1->CCR1 = pulse(config::lora::kPectoralLeftCenter); TIM1->EGR = TIM_EGR_UG;
  }
  __DSB();
}
void fail(FaultReason reason, uint32_t detail, uint32_t pc, uint32_t lr) {
  fault.store(FaultStage::Fatal, std::memory_order_release);
  emergencyNeutral();
  record(reason, detail, pc, lr);
  if (!watchdogArmed || reason == FaultReason::OutputWriteFailure) NVIC_SystemReset();
  for (;;) {} // All other fatal exits stop feeding; hardware PWM can emit neutral first.
}
void controlledReset(FaultReason reason, uint32_t detail) {
  if (faultLatched()) return;
  controlledStartedMs = HAL_GetTick();
  fault.store(FaultStage::ControlledNeutral, std::memory_order_release);
  emergencyNeutral(); record(reason, detail, 0, 0);
  // Two complete 20ms PWM periods before reset. Control still reports progress;
  // a genuinely stalled task/clock is caught by IWDG independently.
}
uint32_t lastFaultReason() { return previousFault; }
uint32_t resetFlags() { return previousReset; }
bool bootConfirmed() { return confirmed; }
}

extern "C" __attribute__((naked)) void HardFault_Handler() {
  __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n b robofish_hard_fault\n");
}
extern "C" void robofish_hard_fault(uint32_t *frame) {
  const uintptr_t address = reinterpret_cast<uintptr_t>(frame);
  const bool valid = !(address & 3U) && address >= RF_RAM_BASE && address <= RF_FAULT_RAM - 8U * 4U;
  robofish::bsp::fail(robofish::bsp::FaultReason::HardFault, SCB->CFSR,
                    valid ? frame[6] : 0, valid ? frame[5] : 0);
  for (;;) {} // If the debugger suppresses reset, do not return to the bad frame.
}
