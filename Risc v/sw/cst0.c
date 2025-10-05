#include <stdint.h>

// --- Symbols from the linker script ---
extern uint32_t _sidata; // Start of .data values in FLASH
extern uint32_t _sdata;  // Start of .data in RAM
extern uint32_t _edata;  // End of .data in RAM
extern uint32_t _sbss;   // Start of .bss in RAM
extern uint32_t _ebss;   // End of .bss in RAM

// --- Function Prototypes ---
int main(void);
void c_startup(void);

void __attribute__((noreturn)) c_startup(void) {

    // Copy .data section from FLASH to RAM
    uint32_t *data_source = &_sidata;
    uint32_t *data_dest = &_sdata;
    while (data_dest < &_edata) {
        *data_dest++ = *data_source++;
    }

    // Clear .bss section in RAM
    uint32_t *bss_dest = &_sbss;
    while (bss_dest < &_ebss) {
        *bss_dest++ = 0;
    }

    // Call main()
    main();
    // Trap if main returns
    while(1) {
        __asm__ volatile ("wfi");
    }
}
