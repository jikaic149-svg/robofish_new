#pragma once
#include <stdint.h>
#include "FreeRTOS.h"
namespace robofish::bsp {
enum class FaultReason : uint32_t { None, Init, Assert, StackOverflow, HardFault, OutputWriteFailure };
inline bool mockFault = false, mockBootConfirmed = true;
inline uint32_t mockFeeds = 0, mockResets = 0, mockEmergencyWrites = 0;
inline FaultReason mockReason = FaultReason::None;
inline bool mockControlled=false;
inline uint32_t mockControlledStart=0;
inline bool faultLatched() { return mockFault; }
inline void initializeSafety() {}
inline void emergencyNeutral() { ++mockEmergencyWrites; }
inline void serviceWatchdog(bool healthy, bool = true) {
  if(mockControlled && uint32_t(mockTick-mockControlledStart)>=40U) {
    ++mockResets;mockControlled=false;return;
  }
  if (healthy && (!mockFault || mockControlled)) ++mockFeeds;
}
inline bool controlledExitActive() {return mockControlled;}
inline void controlledReset(FaultReason reason,uint32_t) {
  mockFault=true;mockReason=reason;mockControlled=true;mockControlledStart=mockTick;emergencyNeutral();
}
inline void fail(FaultReason reason, uint32_t = 0, uint32_t = 0, uint32_t = 0) {
  mockFault = true; mockReason = reason; emergencyNeutral(); ++mockResets;
}
inline uint32_t lastFaultReason() { return static_cast<uint32_t>(mockReason); }
inline uint32_t resetFlags() { return 0; }
inline bool bootConfirmed() { return mockBootConfirmed; }
}
