/* Simple Linker Script for a RISC-V Processor */
ENTRY(_start)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
  RAM   (rwx) : ORIGIN = 0x80000000, LENGTH = 32K
}

SECTIONS
{
  .text :
  {
    KEEP(*(.init))
    *(.text*)
  } > FLASH

  .rodata : ALIGN(4)
  {
    _sidata = LOADADDR(.data);
    *(.rodata*)
  } > FLASH

  .data : ALIGN(4)
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM AT > FLASH

  .bss : ALIGN(4)
  {
    _sbss = .;
    *(.bss*)
    *(COMMON*)
    _ebss = .;
  } > RAM

  _stack_start = ORIGIN(RAM) + LENGTH(RAM);
}
