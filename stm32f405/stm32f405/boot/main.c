#include "stm32f4xx.h"
#include "boot/BootStore.h"

void SystemInit(void) { SCB->VTOR = RF_BOOT_BASE; }
int main(void) {
  rf_boot_record_t record;
  if (!rf_boot_latest(&record, 0)) goto recovery;
  uint32_t valid = 0;
  for (unsigned i = 0; i < 2; ++i)
    if (rf_boot_image_valid(i, &record.image[i])) valid |= 1U << i;
  const uint32_t previous_sequence = record.sequence;
  uint32_t slot = rf_boot_select(&record, valid);
  if (slot == RF_BOOT_NONE) goto recovery;
  if (record.sequence != previous_sequence && !rf_boot_append(&record)) {
    // A trial MUST NOT run unless its used-attempt record has been committed.
    slot = record.confirmed;
    if (!(valid & (1U << slot))) goto recovery;
  }
  const uint32_t base = rf_app_base(slot);
  __disable_irq();
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
  for (unsigned i = 0; i < 8; ++i) { NVIC->ICER[i] = UINT32_C(0xFFFFFFFF); NVIC->ICPR[i] = UINT32_C(0xFFFFFFFF); }
  SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;
  SCB->VTOR = base; __DSB(); __ISB();
  // No stack access after setting MSP. Application starts from a normal reset mask.
  __asm volatile("ldr r1, [%0]\n ldr r2, [%0, #4]\n movs r3, #0\n msr control, r3\n"
                 "msr basepri, r3\n msr faultmask, r3\n msr msp, r1\n cpsie i\n bx r2"
                 : : "r"(base) : "r1", "r2", "r3", "memory");
recovery:
  __disable_irq();
  for (;;) { __WFI(); } // No valid image: never guess a half-written application.
}
