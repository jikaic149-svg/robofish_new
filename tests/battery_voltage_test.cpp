#include <cassert>
#include <cstdio>
#include "algorithm/BatteryVoltage.h"
int main() {
  using robofish::algorithm::batteryVoltage;
  auto v=batteryVoltage(1880,1500,1500,3300,10000,2200);
  assert(v.valid && v.vddaMillivolts==3300 && v.millivolts>=8399 && v.millivolts<=8403);
  auto changed=batteryVoltage(2068,1650,1500,3300,10000,2200);
  assert(changed.valid && changed.vddaMillivolts==3000);
  assert(changed.millivolts==v.millivolts);
  assert(batteryVoltage(0,1500,1500,3300,10000,2200).millivolts==0);
  assert(!batteryVoltage(4096,1500,1500,3300,10000,2200).valid);
  assert(!batteryVoltage(1880,0,1500,3300,10000,2200).valid);
  assert(!batteryVoltage(1880,1500,0xffff,3300,10000,2200).valid);
  assert(!batteryVoltage(1880,1500,1500,3300,10000,0).valid);
  assert(!batteryVoltage(1880,500,1500,3300,10000,2200).valid);
  puts("PASS: divider conversion, factory VREF compensation, zero input and invalid conversion data");
}
