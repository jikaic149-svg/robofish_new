#pragma once
#include <stdint.h>

namespace robofish::bsp {
enum class FaultReason : uint32_t { None, Init, Assert, StackOverflow, HardFault, OutputWriteFailure };
bool faultLatched();
void initializeSafety();
void serviceWatchdog(bool tasksHealthy, bool neutralSelfTest = true);
void controlledReset(FaultReason reason, uint32_t detail);
bool controlledExitActive();
void emergencyNeutral(); // The only write path permitted after a fatal latch.
void fail(FaultReason reason, uint32_t detail = 0, uint32_t pc = 0, uint32_t lr = 0);
uint32_t lastFaultReason();
uint32_t resetFlags();
bool bootConfirmed();
}
extern "C" void robofish_early_watchdog_start();
