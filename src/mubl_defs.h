/*
 * µBL constants shared by board files, the linker script and the C sources.
 * Preprocessor definitions only: this file is also read when generating the linker script.
 */
#ifndef MUBL_DEFS_H
#define MUBL_DEFS_H

/* MCU families (MUBL_FAMILY). */
#define MUBL_FAMILY_HOST 0      /* host simulation, tests only */
#define MUBL_FAMILY_F0 1        /* STM32F0: Cortex-M0, bxCAN, USART v2, 1-2 KiB pages, half-word programming */
#define MUBL_FAMILY_F3 3        /* STM32F3: Cortex-M4, bxCAN, USART v2, 2 KiB pages, half-word programming */
#define MUBL_FAMILY_F4 4        /* STM32F4: Cortex-M4, bxCAN, USART v1, mixed sectors, word programming */
#define MUBL_FAMILY_H7 7        /* STM32H7: Cortex-M7, FDCAN, USART v2, 128 KiB sectors, 256-bit ECC words */

/* Links.  A board lists the ones it has, highest priority first, in MUBL_LINK_PRIORITY. */
#define MUBL_LINK_DRONECAN 1
#define MUBL_LINK_SERIAL 2      /* the PX4/ArduPilot serial bootloader protocol (uploader.py, QGC, MP) */
#define MUBL_LINK_SRXL2 3       /* Spektrum SRXL2: standard handshake plus the µBL update packet */
#define MUBL_LINK_COUNT 4       /* one past the highest identifier */

/* Application image formats the bootloader accepts (MUBL_IMAGE_FORMATS, a mask). */
#define MUBL_IMG_RAW 1          /* no descriptor: plausible vector table, written by µBL (first block last) */
#define MUBL_IMG_LEGACY 2       /* "APDesc00" descriptor, CRC-64-WE over the image (thiemar, early PX4) */
#define MUBL_IMG_AP 4           /* ArduPilot/PX4 unsigned descriptor: split CRC-32 and board ID */

/* Pins: MUBL_PIN('A', 11, 4) is PA11 with alternate function 4 (the AF is ignored for plain I/O). */
#define MUBL_PIN(port, pin, af) ((((port) - 'A') << 8) | ((pin) << 4) | (af))
#define MUBL_PIN_PORT(p) (((p) >> 8) & 0xFu)
#define MUBL_PIN_NUM(p) (((p) >> 4) & 0xFu)
#define MUBL_PIN_AF(p) ((p) & 0xFu)

#endif /* MUBL_DEFS_H */
