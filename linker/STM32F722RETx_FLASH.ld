ENTRY(Reset_Handler)

MEMORY
{
  /* F722RE: 512 KiB flash; sector 7 (128 KiB) is reserved for settings.
   * Flight logs use the external SPI flash. */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 384K
  /* Last 1 KiB is retained across reset for the software DFU request. */
  RAM  (xrw) : ORIGIN = 0x20010000, LENGTH = 191K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x1000;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) _etext = .; } > FLASH
  .ARM.extab : { *(.ARM.extab*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) _edata = .; } > RAM AT> FLASH
  .bss : { _sbss = .; *(.bss*) *(COMMON) _ebss = .; } > RAM
  ._user_heap_stack : {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . += _Min_Heap_Size;
    . += _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM
  /DISCARD/ : { *(.note.GNU-stack) }
}
