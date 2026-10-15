/* X Pulse STM32G474VET6: PC2 red, PC1 green, PC0 blue, active low. */
#include <stdint.h>
#define REG(address) (*(volatile uint32_t *)(address))
extern uint32_t _sbss, _ebss;
volatile uint32_t milliseconds;
volatile uint32_t colour_index;
static const uint8_t colours[] = {4, 2, 1};
void Reset_Handler(void);
void SysTick_Handler(void);
void Fault_Handler(void) { for (;;) __asm volatile ("wfi"); }
__attribute__((section(".vectors"),used))
const uintptr_t vectors[16] = {
    0x20004000, (uintptr_t)Reset_Handler,
    (uintptr_t)Fault_Handler, (uintptr_t)Fault_Handler,
    (uintptr_t)Fault_Handler, (uintptr_t)Fault_Handler, (uintptr_t)Fault_Handler,
    0,0,0,0,(uintptr_t)Fault_Handler,(uintptr_t)Fault_Handler,0,
    (uintptr_t)Fault_Handler,(uintptr_t)SysTick_Handler
};
static void show(uint32_t index) {
    uint32_t low=colours[index];
    REG(0x48000818)=(7U & ~low) | (low << 16); /* GPIOC BSRR */
    colour_index=index;
}
void SysTick_Handler(void) {
    if (++milliseconds % 800U == 0) show((colour_index+1U)%3U);
}
void Reset_Handler(void) {
    __asm volatile ("cpsid i");
    for (uint32_t *p=&_sbss;p<&_ebss;++p) *p=0;
    REG(0xE000ED08)=(uint32_t)vectors; /* VTOR */
    REG(0x4002104C) |= 1U<<2; /* RCC AHB2ENR: GPIOC */
    (void)REG(0x4002104C);
    REG(0x48000818)=7U;
    REG(0x48000804)&=~7U; /* push-pull */
    REG(0x4800080C)&=~0x3FU;
    REG(0x48000800)=(REG(0x48000800)&~0x3FU)|0x15U;
    show(0);
    /* Hardware reset defaults: HSI16, AHB divide by one. */
    REG(0xE000E014)=16000U-1U;
    REG(0xE000E018)=0;
    REG(0xE000E010)=7U;
    __asm volatile ("dsb\nisb\ncpsie i");
    for (;;) __asm volatile ("wfi");
}
