#ifndef RF_STM32_BOOT_H
#define RF_STM32_BOOT_H
#include <stdint.h>
#include <stddef.h>

#define RF_BOOT_BASE UINT32_C(0x08000000)
#define RF_BOOT_END UINT32_C(0x08008000)
#define RF_RECORD0 UINT32_C(0x08008000)
#define RF_RECORD1 UINT32_C(0x0800C000)
#define RF_RECORD_BYTES UINT32_C(0x4000)
#define RF_APP_A UINT32_C(0x08010000)
#define RF_APP_B UINT32_C(0x08080000)
#define RF_APP_MAX_BYTES UINT32_C(0x70000)
#define RF_RAM_BASE UINT32_C(0x20000000)
#define RF_FAULT_RAM UINT32_C(0x2001FFC0)
#define RF_VECTOR_BYTES 392U
#define RF_RECORD_STRIDE 128U
#define RF_BOOT_MAGIC UINT32_C(0x52464231)
#define RF_BOOT_FORMAT 1U
#define RF_BOOT_COMMIT UINT32_C(0xA55AA55A)
#define RF_BOOT_NONE UINT32_C(0xFFFFFFFF)
#define RF_BOOT_STABLE 0U
#define RF_BOOT_PENDING 1U
#define RF_BOOT_TRYING 2U
#define RF_FAULT_MAGIC UINT32_C(0x52464631)
#define RF_FAULT_COMMIT UINT32_C(0x5AA55AA5)

// F405 native CRC: polynomial 04C11DB7, init FFFFFFFF, no reflection/XOR.
// Input words are little-endian loads, processed MSB first; short tail pads FF.
static inline uint32_t rf_crc_word(uint32_t crc, uint32_t word) {
  crc ^= word;
  for (unsigned bit = 0; bit < 32; ++bit)
    crc = (crc << 1) ^ ((crc & UINT32_C(0x80000000)) ? UINT32_C(0x04C11DB7) : 0U);
  return crc;
}
static inline uint32_t rf_crc_bytes(const void *data, size_t length) {
  const uint8_t *p = (const uint8_t *)data;
  uint32_t crc = UINT32_C(0xFFFFFFFF);
  for (size_t i = 0; i < length; i += 4) {
    uint32_t word = 0;
    for (unsigned b = 0; b < 4; ++b)
      word |= (uint32_t)(i + b < length ? p[i + b] : 0xFFU) << (8U * b);
    crc = rf_crc_word(crc, word);
  }
  return crc;
}
typedef struct { uint32_t size, crc; uint8_t sha256[32]; } rf_boot_image_t;
typedef struct {
  uint32_t magic, format, sequence, confirmed, candidate, state, attempts, confirmed_mask;
  rf_boot_image_t image[2];
  uint32_t crc, commit;
} rf_boot_record_t;
typedef struct {
  uint32_t magic, version, reason, pc, lr, detail, reset_flags, sequence;
  uint32_t tick, reserved[5];
  uint32_t crc, commit;
} rf_fault_record_t;
static inline uint32_t rf_app_base(uint32_t slot) { return slot ? RF_APP_B : RF_APP_A; }
static inline int rf_boot_record_valid(const rf_boot_record_t *r) {
  if (r->commit != RF_BOOT_COMMIT || r->magic != RF_BOOT_MAGIC ||
      r->format != RF_BOOT_FORMAT || r->confirmed > 1 || r->state > RF_BOOT_TRYING ||
      (r->confirmed_mask & ~3U) || !(r->confirmed_mask & (1U << r->confirmed))) return 0;
  if (r->state != RF_BOOT_STABLE && (r->candidate > 1 || r->candidate == r->confirmed)) return 0;
  if (r->state != RF_BOOT_STABLE && (r->confirmed_mask & (1U << r->candidate))) return 0;
  if (r->state == RF_BOOT_STABLE && r->candidate != RF_BOOT_NONE) return 0;
  for (unsigned i = 0; i < 2; ++i)
    if ((r->confirmed_mask & (1U << i)) || (r->state != RF_BOOT_STABLE && r->candidate == i))
      if (r->image[i].size < RF_VECTOR_BYTES || r->image[i].size > RF_APP_MAX_BYTES) return 0;
  return r->crc == rf_crc_bytes(r, offsetof(rf_boot_record_t, crc));
}
static inline int rf_generation_after(uint32_t a, uint32_t b) {
  return a != b && (uint32_t)(a - b) < UINT32_C(0x80000000);
}
static inline int rf_app_vectors_valid(uint32_t base, uint32_t size, uint32_t sp, uint32_t pc) {
  return (base == RF_APP_A || base == RF_APP_B) &&
         size >= RF_VECTOR_BYTES && size <= RF_APP_MAX_BYTES &&
         sp > RF_RAM_BASE && sp <= RF_FAULT_RAM && !(sp & 7U) &&
         (pc & 1U) && (pc & ~1U) >= base + RF_VECTOR_BYTES && (pc & ~1U) - base < size;
}
#if defined(__cplusplus)
static_assert(sizeof(rf_boot_record_t) == 120 && sizeof(rf_fault_record_t) == 64, "Boot ABI");
#else
_Static_assert(sizeof(rf_boot_record_t) == 120 && sizeof(rf_fault_record_t) == 64, "Boot ABI");
#endif
#endif
