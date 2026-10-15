#pragma once
class MotionController;
namespace robofish::bsp {
bool startBatteryAdc(MotionController &controller);
void pollBatteryAdc();
}
