#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
const uint8_t *rf_host_flash(uint32_t address);
int rf_host_erase(uint32_t page);
int rf_host_program(uint32_t address, uint32_t value);
#ifdef __cplusplus
}
#endif
