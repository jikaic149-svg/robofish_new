#pragma once
#include <stdint.h>
namespace robofish::algorithm {
struct BatteryVoltage { uint16_t millivolts = 0, vddaMillivolts = 0; bool valid = false; };
// Factory VREFINT calibration and divider values are supplied by the BSP.
inline BatteryVoltage batteryVoltage(uint16_t batteryRaw, uint16_t referenceRaw,
    uint16_t factoryReference, uint32_t referenceMv, uint32_t upperOhms, uint32_t lowerOhms) {
  if (batteryRaw > 4095 || referenceRaw == 0 || referenceRaw > 4095 ||
      factoryReference == 0 || factoryReference > 4095 || !referenceMv || !lowerOhms)
    return {};
  const uint64_t reference = uint64_t(referenceMv) * factoryReference;
  const uint64_t supply = (reference + referenceRaw / 2) / referenceRaw;
  if (supply < 1800 || supply > 3600) return {};
  const uint64_t denominator = uint64_t(referenceRaw) * 4095 * lowerOhms;
  const uint64_t numerator = reference * batteryRaw * (uint64_t(upperOhms) + lowerOhms);
  const uint64_t battery = (numerator + denominator / 2) / denominator;
  if (battery > 65535) return {};
  return {uint16_t(battery), uint16_t(supply), true};
}
}
