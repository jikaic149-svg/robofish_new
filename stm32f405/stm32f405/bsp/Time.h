#pragma once
#include <stdint.h>
namespace robofish::bsp {
uint32_t nowMs(); // HAL monotonic milliseconds, wrapping at uint32_t.
}
