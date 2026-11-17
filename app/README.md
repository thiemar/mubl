# Applications for µBL

An application runs from `MUBL_APP_BASE` (the flash right after the bootloader) and is started
like after a reset: its vector table is in place, the stack pointer is loaded from its first word,
interrupts are enabled (PRIMASK clear) with every NVIC source disabled and nothing pending,
SysTick is off, the peripherals and GPIO ports the bootloader used are back in their reset state,
and the clocks are back on the internal oscillator (on H7 the supply configuration has been
written; see below).

## Memory

| | F0 (Cortex-M0) | F3, F4, H7 |
| --- | --- | --- |
| Flash origin | `MUBL_APP_BASE` | `MUBL_APP_BASE` |
| Vector table | copied by the bootloader to the start of SRAM, which is mapped at address 0 (SYSCFG `MEM_MODE`) | `VTOR` set to `MUBL_APP_BASE` |
| Reserved RAM | `MUBL_RAM_RESERVED` bytes at the start of RAM: 0xC0 of vectors, then the handoff block | `MUBL_RAM_RESERVED` bytes at the start of RAM: the handoff block (after AP_Periph's 256 bytes with `MUBL_COMPAT_AP_PERIPH`) |

The application must keep `.data`, `.bss`, heap and stack out of the reserved RAM; for a board with
the defaults that is 256 bytes (0x1C0 on F0) from the start of RAM:

```
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08004000, LENGTH = 1008K   /* MUBL_APP_BASE, MUBL_APP_SIZE */
    RAM (rwx)   : ORIGIN = 0x20000100, LENGTH = 128K - 0x100
}
```

On F0 the application may change its vectors at run time by writing the copy in SRAM.

The initial stack pointer (the first word of the image) must point into RAM the bootloader's board
file declares: `MUBL_RAM_BASE`/`SIZE` or a bank in `MUBL_STACK_RAM`.  If the image has a descriptor,
the reset vector must point inside the image it describes.

## Descriptor

With the default image formats the bootloader boots only an image carrying a descriptor whose CRCs
and board ID match.  Put an ArduPilot/PX4-format descriptor right after the vector table:

```c
struct app_descriptor {
    uint8_t sig[8];
    uint32_t image_crc1, image_crc2, image_size, git_hash;
    uint8_t version_major, version_minor;
    uint16_t board_id;
    uint8_t reserved[8];
};
__attribute__((section(".app_descriptor"), used))
const struct app_descriptor app_descriptor = {
    {0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06}, 0, 0, 0, GIT_HASH, 1, 0, MUBL_BOARD_ID, {0}};
```

```
.text : { KEEP(*(.isr_vector)) KEEP(*(.app_descriptor)) *(.text*) ... } > FLASH
```

then fill in its size and CRCs after linking:

```
tools/mubl_image.py stamp app.bin                # in place; prints the result
tools/mubl_image.py apj app.bin --board-id N     # for uploader.py, QGroundControl, Mission Planner
```

The legacy `APDesc00` descriptor (thiemar, early PX4: CRC-64 over the image) is accepted too, and
`stamp` fills it the same way.  `MUBL_IMG_RAW` accepts images with no descriptor; see the README
for what that gives up.

## Handoff

`app/mubl_app.h` reads what the bootloader passed, confirms a good boot and asks the bootloader to
stay (for example when a DroneCAN BeginFirmwareUpdate arrives):

```c
#define MUBL_HANDOFF_ADDR 0x20000000          /* the bootloader board's value */
#include "mubl_app.h"

mubl_info_t info;
if (mubl_app_info(&info)) {                   /* early: node ID and bitrate the bootloader found */
    can_init(info.can_bitrate);
    node_id = info.can_node_id;
}
...
mubl_app_confirm();                           /* once running properly */
...
/* BeginFirmwareUpdate from node `server` for `path`: answer it, then */
mubl_app_request(MUBL_LINK_DRONECAN, node_id, bitrate, server, path, 0, 0);
NVIC_SystemReset();
```

`mubl_app_confirm` matters only on boards with `MUBL_MAX_BOOT_ATTEMPTS`: there, after that many
boots that were not confirmed (watchdog resets, crashes), the bootloader stays so the device can be
updated.  Existing AP_Periph and PX4 applications need no changes on boards with
`MUBL_COMPAT_AP_PERIPH` or `MUBL_COMPAT_PX4_CAN`.

## Things to know

- **Watchdog.**  On boards with `MUBL_WATCHDOG`, or with the hardware-watchdog option byte set, the
  IWDG is running with a period of `MUBL_WATCHDOG_MS` (8 s) when the application starts, and cannot
  be stopped; the application may change the period.  Otherwise the bootloader leaves it untouched.
- **H7 supply.**  The bootloader writes `PWR_CR3` (`MUBL_H7_PWR_CR3`), which can be written only once
  per power-on.  The application must configure the same supply (Cube's `HAL_PWREx_ConfigSupply`
  returns an error for a different one).
- **Clocks.**  The bootloader returns to HSI with the PLL and HSE off (H7: HSE off; the CPU never left
  HSI), so the application's clock set-up starts from the reset configuration.
