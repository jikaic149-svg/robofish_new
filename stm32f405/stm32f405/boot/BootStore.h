#pragma once
#include "stm32_boot.h"
#ifdef __cplusplus
extern "C" {
#endif
int rf_boot_latest(rf_boot_record_t *record, uint32_t *address);
int rf_boot_append(const rf_boot_record_t *record);
int rf_boot_image_valid(uint32_t slot, const rf_boot_image_t *image);
// Pure selection: modified record must be persisted BEFORE a trial jump.
uint32_t rf_boot_select(rf_boot_record_t *record, uint32_t valid_images);
#ifdef __cplusplus
}
#endif
