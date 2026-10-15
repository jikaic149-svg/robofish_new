ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}
SECTIONS {
  .vectors : { KEEP(*(.vectors)) } > FLASH
  .text : { *(.text*) *(.rodata*) } > FLASH
  .data : { *(.data*) } > RAM AT> FLASH
  ASSERT(SIZEOF(.data) == 0, "This minimal startup requires no initialized RAM data")
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
  ASSERT(_ebss < 0x20003000, "Insufficient stack")
  /DISCARD/ : { *(.comment) *(.ARM.exidx*) *(.ARM.extab*) }
}
