/*
 * Host simulation board (tests).  SIM_GEOMETRY picks the flash a family has:
 *   0  STM32F0-like: 1 KiB pages, half-word programming, raw images accepted
 *   1  STM32F4-like: 16/64/128 KiB sectors, word programming
 *   2  STM32H7-like: 128 KiB sectors, 32-byte ECC flash words
 *   3  STM32G4-like: 2 KiB pages, 8-byte ECC double words
 * Tests rebuild the library with other -D settings (priority, exclusive links, ...).
 */
#define MUBL_SIM 1
#define MUBL_FAMILY MUBL_FAMILY_HOST

#ifndef SIM_GEOMETRY
#define SIM_GEOMETRY 1
#endif

#if SIM_GEOMETRY == 0
#define MUBL_FLASH_GRANULE 2
#define MUBL_FLASH_ECC 0
#define SIM_FLASH_SIZE (64U * 1024U)
#define SIM_SECTOR 1024U
#define MUBL_BL_SIZE (8 * 1024)
#ifndef MUBL_CRC32_TABLE
#define MUBL_CRC32_TABLE 0             /* as on F0: both CRCs bit by bit */
#endif
#ifndef MUBL_CRC64_TABLE
#define MUBL_CRC64_TABLE 0
#endif
#define MUBL_VECTOR_RESERVE 0xC0
#ifndef MUBL_IMAGE_FORMATS
#define MUBL_IMAGE_FORMATS (MUBL_IMG_RAW | MUBL_IMG_LEGACY | MUBL_IMG_AP)
#endif
#elif SIM_GEOMETRY == 1
#define MUBL_FLASH_GRANULE 4
#define MUBL_FLASH_ECC 0
#define SIM_FLASH_SIZE (512U * 1024U)
#define MUBL_BL_SIZE (16 * 1024)
#elif SIM_GEOMETRY == 3
#define MUBL_FLASH_GRANULE 8
#define MUBL_FLASH_ECC 1
#define SIM_FLASH_SIZE (256U * 1024U)
#define SIM_SECTOR 2048U
#define MUBL_BL_SIZE (16 * 1024)
#else
#define MUBL_FLASH_GRANULE 32
#define MUBL_FLASH_ECC 1
#define SIM_FLASH_SIZE (1024U * 1024U)
#define SIM_SECTOR (128U * 1024U)
#define MUBL_BL_SIZE (128 * 1024)
#endif

#define MUBL_APP_SIZE (SIM_FLASH_SIZE - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000
#ifndef MUBL_RAM_SIZE
#define MUBL_RAM_SIZE (64 * 1024)
#endif
#define MUBL_STACK_RAM {0x10000000, 0x10002000}   /* an 8 KiB CCM-like bank */

#ifndef MUBL_BOARD_ID
#define MUBL_BOARD_ID 9999
#endif
#define MUBL_BOARD_NAME "org.sfwa.mubl.sim"

#ifndef MUBL_DRONECAN
#define MUBL_DRONECAN 1
#endif
#ifndef MUBL_SERIAL
#define MUBL_SERIAL 1
#endif
#ifndef MUBL_SRXL2
#define MUBL_SRXL2 1
#endif
#ifndef MUBL_USB
#define MUBL_USB 1
#endif
#define MUBL_UART0_INSTANCE 1     /* port 0: single wire, shared by SRXL2 and serial by default */
#define MUBL_UART1_INSTANCE 2
#ifndef MUBL_COMPAT_AP_PERIPH
#define MUBL_COMPAT_AP_PERIPH 1
#endif
#ifndef MUBL_COMPAT_PX4_CAN
#define MUBL_COMPAT_PX4_CAN 1
#endif
#ifndef MUBL_MAX_BOOT_ATTEMPTS
#define MUBL_MAX_BOOT_ATTEMPTS 3
#endif
