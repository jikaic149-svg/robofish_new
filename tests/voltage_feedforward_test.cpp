#include <cassert>
#include <cstdio>
#include "device/motion/EscCalibration.h"
#include "channel/BatterySample.h"
int main() {
 using robofish::device::voltageCompensatedPropulsionPercent;
 // Changing real input is independently compensated at the same supply.
 assert(voltageCompensatedPropulsionPercent(20,7400,8000)==19);
 assert(voltageCompensatedPropulsionPercent(60,7400,8000)==56);
 assert(voltageCompensatedPropulsionPercent(50,7400,8400)==44);
 assert(voltageCompensatedPropulsionPercent(50,7400,6800)==54);
 assert(voltageCompensatedPropulsionPercent(-50,7400,6800)==-54);
 assert(voltageCompensatedPropulsionPercent(0,7400,6800)==0);
 assert(voltageCompensatedPropulsionPercent(90,7400,6600)==100);
 assert(voltageCompensatedPropulsionPercent(-90,7400,6600)==-100);
 assert(voltageCompensatedPropulsionPercent(50,7400,0)==0);
 assert(voltageCompensatedPropulsionPercent(50,0,6800)==0);
 for(int input=-100;input<=100;++input)
  assert(voltageCompensatedPropulsionPercent(input,7400,7400)==input);
 for(unsigned cycle=0;cycle<100;++cycle)
  assert(voltageCompensatedPropulsionPercent(50,7400,8000)==46);
 assert(robofish::device::propulsionPulseUs(0)==robofish::config::lora::kEscNeutralUs);
 robofish::BatterySample sample{1,1000,7400,3300,1656,true,7400};
 assert(robofish::usableBatteryMillivolts(sample,1500,500)==7400);
 assert(robofish::usableBatteryMillivolts(sample,1501,500)==0);
 sample.valid=false;assert(robofish::usableBatteryMillivolts(sample,1000,500)==0);
 sample.valid=true;sample.sampledAtMs=0xfffffff0U;
 assert(robofish::usableBatteryMillivolts(sample,0x10U,500)==7400);
 sample.millivolts=0;assert(robofish::usableBatteryMillivolts(sample,0x10U,500)==0);
 using robofish::BatteryCondition;
 for (uint16_t voltage : {uint16_t(0), uint16_t(4), uint16_t(6599), uint16_t(8401), uint16_t(65535)}) {
  assert(voltageCompensatedPropulsionPercent(20,7400,voltage)==0);
  assert(voltageCompensatedPropulsionPercent(-20,7400,voltage)==0);
 }
 sample = {2,1000,7400,3300,1,true,4};
 assert(robofish::batteryCondition(sample,1000,500)==BatteryCondition::Low);
 assert(robofish::usableBatteryMillivolts(sample,1000,500)==0);
 sample.rawMillivolts=8401;
 assert(robofish::batteryCondition(sample,1000,500)==BatteryCondition::High);
 for (auto boundary : {uint16_t(6600),uint16_t(8400)}) {
  sample.rawMillivolts=sample.millivolts=boundary;
  assert(robofish::batteryCondition(sample,1500,500)==BatteryCondition::Normal);
 }
 assert(robofish::batteryCondition(sample,1501,500)==BatteryCondition::Unavailable);
 sample.valid=false;
 assert(robofish::batteryCondition(sample,1000,500)==BatteryCondition::Unavailable);
 puts("PASS: dynamic signed input, varying supply, limits, neutral, invalid data and no recursive compensation");
}
