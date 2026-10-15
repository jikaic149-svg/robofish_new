ENTRY(Reset_Handler)
MEMORY
{
  FLASH (rx) : ORIGIN = @RF_LINK_BASE@, LENGTH = @RF_LINK_LENGTH@
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 0x1FFC0
  RETAIN (rw) : ORIGIN = 0x2001FFC0, LENGTH = 64
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Stack_Size = 0x1000;
SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
  .text :
  {
    . = ALIGN(4);
    *(.text .text.*) *(.glue_7 .glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4); _etext = .;
  } > FLASH
  .rodata : { . = ALIGN(4); *(.rodata .rodata.*) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx :
  {
    __exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
  } > FLASH
  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4); _sdata = .;
    *(.data .data.*) . = ALIGN(4); _edata = .;
  } > RAM AT > FLASH
  .bss (NOLOAD) :
  {
    . = ALIGN(4); _sbss = .; __bss_start__ = .;
    *(.bss .bss.*) *(COMMON)
    . = ALIGN(4); _ebss = .; __bss_end__ = .;
  } > RAM
  .noinit (NOLOAD) :
  {
    . = ALIGN(8); KEEP(*(.noinit.fault))
  } > RETAIN
  .stack_reservation (NOLOAD) :
  {
    . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .);
    . += _Min_Stack_Size; . = ALIGN(8);
  } > RAM
  _ram_usage_end = .;
  .ARM.attributes 0 : { *(.ARM.attributes) }
}

ASSERT(ADDR(.isr_vector) == ORIGIN(FLASH), "Vector table must start at slot base")
ASSERT(SIZEOF(.isr_vector) <= 512, "Vector table exceeds alignment")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(FLASH) + LENGTH(FLASH), "Flash load image exceeds region")
ASSERT(_ram_usage_end <= ORIGIN(RAM) + LENGTH(RAM), "RAM/stack overlaps retained fault record")
ASSERT(SIZEOF(.noinit) <= LENGTH(RETAIN), "Fault record exceeds retained RAM")
