/* h735g-dk bootloader: application at MUBL_APP_BASE, RAM after the 256-byte handoff block. */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08020000, LENGTH = 896K
    RAM (rwx)  : ORIGIN = 0x20000100, LENGTH = 128K - 0x100
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
ENTRY(reset_handler)
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
