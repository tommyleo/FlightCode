ENTRY(Reset_Handler)

MEMORY
{
  /* Bank 2 sectors 6 and 7 are reserved for flight log and settings. */
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1792K
  /* The final 1 KiB remains untouched for the reset-to-DFU request. */
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 127K
  RAM (xrw)   : ORIGIN = 0x24000000, LENGTH = 511K
}

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size = 0x400;
_Min_Stack_Size = 0x2000;

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
  } > DTCMRAM
  /DISCARD/ : { *(.note.GNU-stack) }
}
