#include "boot/BootStore.h"
#include <string.h>
#if defined(RF_BOOT_HOST_TEST)
#include "boot_storage_stub.h"
#define MAP(address) rf_host_flash(address)
#define ERASE(page) rf_host_erase(page)
#define PROGRAM(address,value) rf_host_program(address,value)
#else
#include "stm32f4xx.h"
#define MAP(address) ((const uint8_t *)(uintptr_t)(address))
#define ERRORS (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)
static int wait_flash(void) {
  // Boot/confirmation only, never a running motion loop; bounded fallback.
  uint32_t budget = 16000000U;
  while ((FLASH->SR & FLASH_SR_BSY) && --budget) {}
  return budget != 0U && !(FLASH->SR & ERRORS);
}
static void flush_cache(void) {
  uint32_t acr = FLASH->ACR;
  FLASH->ACR = acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
  FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
  FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
  FLASH->ACR = acr;
  __DSB(); __ISB();
}
static int erase_page(uint32_t page) {
  if (page != RF_RECORD0 && page != RF_RECORD1) return 0;
  if (!wait_flash()) return 0;
  FLASH->SR = ERRORS | FLASH_SR_EOP;
  FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB | FLASH_CR_PSIZE)) |
      FLASH_CR_SER | FLASH_CR_PSIZE_1 | ((page == RF_RECORD0 ? 2U : 3U) << 3);
  FLASH->CR |= FLASH_CR_STRT;
  int ok = wait_flash();
  FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  flush_cache(); return ok;
}
static int program_word(uint32_t address, uint32_t value) {
  if (address < RF_RECORD0 || address >= RF_RECORD1 + RF_RECORD_BYTES || (address & 3U)) return 0;
  volatile uint32_t *p = (volatile uint32_t *)(uintptr_t)address;
  if ((*p & value) != value || !wait_flash()) return 0;
  FLASH->SR = ERRORS | FLASH_SR_EOP;
  FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE) | FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  *p = value;
  int ok = wait_flash(); FLASH->CR &= ~FLASH_CR_PG;
  flush_cache(); return ok && *p == value;
}
#define ERASE(page) erase_page(page)
#define PROGRAM(address,value) program_word(address,value)
#endif

static void read_record(uint32_t address, rf_boot_record_t *out) {
  const volatile uint32_t *p = (const volatile uint32_t *)MAP(address);
  for (unsigned i = 0; i < sizeof(*out) / 4; ++i) {
    uint32_t word = p[i]; memcpy((uint8_t *)out + i * 4, &word, 4);
  }
}
int rf_boot_latest(rf_boot_record_t *out, uint32_t *address) {
  int found = 0; rf_boot_record_t candidate;
  for (unsigned page = 0; page < 2; ++page) {
    const uint32_t base = page ? RF_RECORD1 : RF_RECORD0;
    for (unsigned offset = 0; offset < RF_RECORD_BYTES; offset += RF_RECORD_STRIDE) {
      const uint32_t a = base + offset;
      if (*(const volatile uint32_t *)MAP(a + offsetof(rf_boot_record_t, commit)) != RF_BOOT_COMMIT) continue;
      read_record(a, &candidate);
      if (!rf_boot_record_valid(&candidate)) continue;
      if (!found || rf_generation_after(candidate.sequence, out->sequence)) {
        *out = candidate; if (address) *address = a; found = 1;
      }
    }
  }
  return found;
}
static uint32_t erased_slot(uint32_t page) {
  for (unsigned offset = 0; offset < RF_RECORD_BYTES; offset += RF_RECORD_STRIDE) {
    const volatile uint32_t *p = (const volatile uint32_t *)MAP(page + offset);
    unsigned i = 0;
    while (i < RF_RECORD_STRIDE / 4 && p[i] == UINT32_C(0xFFFFFFFF)) ++i;
    if (i == RF_RECORD_STRIDE / 4) return page + offset;
  }
  return 0;
}
int rf_boot_append(const rf_boot_record_t *record) {
  rf_boot_record_t next = *record, current, check;
  next.magic = RF_BOOT_MAGIC; next.format = RF_BOOT_FORMAT;
  next.crc = rf_crc_bytes(&next, offsetof(rf_boot_record_t, crc)); next.commit = RF_BOOT_COMMIT;
  if (!rf_boot_record_valid(&next)) return 0;
  uint32_t current_address = 0;
  int have_current = rf_boot_latest(&current, &current_address);
  if (have_current && !rf_generation_after(next.sequence, current.sequence)) return 0;
  uint32_t page = current_address >= RF_RECORD1 ? RF_RECORD1 : RF_RECORD0;
  uint32_t destination = erased_slot(page);
#if !defined(RF_BOOT_HOST_TEST)
  uint32_t primask = __get_PRIMASK(); __disable_irq();
  if (FLASH->CR & FLASH_CR_LOCK) { FLASH->KEYR = UINT32_C(0x45670123); FLASH->KEYR = UINT32_C(0xCDEF89AB); }
#endif
  int ok = 0;
  if (!destination) {
    page = page == RF_RECORD0 ? RF_RECORD1 : RF_RECORD0;
    destination = erased_slot(page);
    if (!destination) {
      if (!ERASE(page)) goto finish;
      destination = page;
    }
  }
  for (unsigned i = 0; i < offsetof(rf_boot_record_t, commit) / 4; ++i) {
    uint32_t word; memcpy(&word, (const uint8_t *)&next + i * 4, 4);
    if (!PROGRAM(destination + i * 4, word)) goto finish;
  }
  read_record(destination, &check); check.commit = RF_BOOT_COMMIT;
  if (memcmp(&check, &next, sizeof(next)) != 0 || !rf_boot_record_valid(&check)) goto finish;
  if (!PROGRAM(destination + offsetof(rf_boot_record_t, commit), RF_BOOT_COMMIT)) goto finish;
  read_record(destination, &check); ok = rf_boot_record_valid(&check);
finish:
#if !defined(RF_BOOT_HOST_TEST)
  FLASH->CR |= FLASH_CR_LOCK;
  if (!primask) __enable_irq();
#endif
  return ok;
}
int rf_boot_image_valid(uint32_t slot, const rf_boot_image_t *image) {
  if (slot > 1 || image->size < RF_VECTOR_BYTES || image->size > RF_APP_MAX_BYTES) return 0;
  uint32_t base = rf_app_base(slot), vectors[2];
  memcpy(vectors, MAP(base), sizeof(vectors));
  if (!rf_app_vectors_valid(base, image->size, vectors[0], vectors[1])) return 0;
  for (unsigned i = 2; i < RF_VECTOR_BYTES / 4; ++i) {
    uint32_t handler; memcpy(&handler, MAP(base + i * 4), 4);
    if (handler && (!(handler & 1U) || (handler & ~1U) < base + RF_VECTOR_BYTES ||
                    (handler & ~1U) - base >= image->size)) return 0;
  }
#if defined(RF_BOOT_HOST_TEST)
  return image->crc == rf_crc_bytes(MAP(base), image->size);
#else
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  (void)RCC->AHB1ENR;
  CRC->CR = CRC_CR_RESET;
  const uint8_t *bytes = MAP(base);
  for (uint32_t offset = 0; offset < image->size; offset += 4) {
    uint32_t word = 0;
    for (unsigned b = 0; b < 4; ++b)
      word |= (uint32_t)(offset + b < image->size ? bytes[offset + b] : 0xFFU) << (8U * b);
    CRC->DR = word;
  }
  return CRC->DR == image->crc;
#endif
}
uint32_t rf_boot_select(rf_boot_record_t *r, uint32_t valid) {
  if (r->state == RF_BOOT_PENDING && r->attempts < 1 && (valid & (1U << r->candidate))) {
    ++r->sequence; ++r->attempts; r->state = RF_BOOT_TRYING;
    return r->candidate;
  }
  uint32_t chosen = r->confirmed;
  if (!(valid & (1U << chosen))) {
    chosen ^= 1U;
    if (!(valid & r->confirmed_mask & (1U << chosen))) return RF_BOOT_NONE;
  }
  if (r->state != RF_BOOT_STABLE || chosen != r->confirmed) {
    ++r->sequence; r->confirmed = chosen; r->state = RF_BOOT_STABLE;
    r->candidate = RF_BOOT_NONE;
  }
  return chosen;
}
