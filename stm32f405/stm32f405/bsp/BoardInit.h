#pragma once
namespace robofish::bsp {
// Called once before task creation; all MCU clocks, pins, DMA and PWM live here.
void initializeBoard();
}
