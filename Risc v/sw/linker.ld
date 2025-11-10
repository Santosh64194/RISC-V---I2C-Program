/* Linker Script for a RISC-V Processor */
ENTRY(_start)
MEMORY
{
  imem (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
  dmem   (rwx) : ORIGIN = 0x80000000, LENGTH = 1K
}
SECTIONS
{
  .text :
  {
    KEEP(*(.init))
    *(.text*)
  } > imem
  .data : ALIGN(4)
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > dmem AT > imem
  .bss : ALIGN(4)
  {
    _sbss = .;
    *(.bss*)
    *(COMMON*)
    _ebss = .;
  } > dmem
  _stack_start = ORIGIN(dmem) + LENGTH(dmem);
}
