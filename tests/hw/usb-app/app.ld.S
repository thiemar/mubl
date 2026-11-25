/* The test application: at MUBL_APP_BASE, RAM after the bootloader's reserved block (handoff). */
#include "board.h"
#include "mubl_config.h"

MEMORY
{
    FLASH (rx) : ORIGIN = MUBL_APP_BASE, LENGTH = MUBL_APP_SIZE
    RAM (rwx) : ORIGIN = MUBL_RAM_BASE + MUBL_RAM_RESERVED, LENGTH = MUBL_RAM_SIZE - MUBL_RAM_RESERVED
}
ENTRY(reset_handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{
    .text : {
        KEEP(*(.isr_vector))
        KEEP(*(.app_descriptor))
        *(.text*) *(.rodata*)
        . = ALIGN(4);
    } > FLASH
    _sidata = LOADADDR(.data);
    .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
    .bss : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
}
