/*
 * µBL linker script, run through the C preprocessor with the board's configuration.
 * The bootloader's RAM starts after the reserved block (application vectors on Cortex-M0, the
 * AP_Periph structure, the handoff block) so start-up never clears what the application left.
 */
#include "board.h"
#include "mubl_config.h"

#ifdef MUBL_MEASURE
#define FLASH_LENGTH 0x40000           /* tools/sizes.py: report the size even when it would not fit */
#else
#define FLASH_LENGTH MUBL_BL_SIZE
#endif

MEMORY
{
    FLASH (rx) : ORIGIN = MUBL_FLASH_BASE, LENGTH = FLASH_LENGTH
    RAM (rwx) : ORIGIN = MUBL_RAM_BASE + MUBL_RAM_RESERVED, LENGTH = MUBL_RAM_SIZE - MUBL_RAM_RESERVED
}

ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > FLASH
    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    .bss (NOLOAD) : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM
    /DISCARD/ : { *(.ARM.attributes) *(.comment) }
}

ASSERT(_estack - _ebss >= MUBL_MIN_STACK, "less than MUBL_MIN_STACK bytes left for the stack")
ASSERT(MUBL_APP_BASE == MUBL_FLASH_BASE + MUBL_BL_SIZE, "the application must follow the bootloader")
