.section .init
.global _start

_start:
  // Set stack pointer
  la sp, _stack_start

  // Jump to C startup
  call c_startup

_halt:
  j _halt
