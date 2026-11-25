/*
 * ST STM32H735G-DK discovery kit (STM32H735IGK6): 1 MiB flash, direct SMPS supply (as every ST
 * example for the kit: USE_PWR_DIRECT_SMPS_SUPPLY), 25 MHz crystal (unused: USB runs from HSI48,
 * trimmed by the CRS from the host's start-of-frame packets).
 *
 * USB only, on the kit's USB OTG connector (OTG_HS with its full-speed PHY on PA11/PA12, no VBUS
 * sensing, as ST's DFU_Standalone example); not the ST-LINK's virtual COM port.  The application
 * starts at 128 KiB (sector 1), where that example puts it too.  LED1 (green, PC3, active low) is
 * on in the bootloader; the user button B1 (PC13, active high) held at reset keeps it running.
 */
#define MUBL_FAMILY MUBL_FAMILY_H7
#define MUBL_BOARD_NAME "org.sfwa.mubl.h735g-dk"
#define MUBL_BOARD_ID 7352

#define MUBL_BL_SIZE (128 * 1024)
#define MUBL_APP_SIZE (1024 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000                 /* DTCM */
#define MUBL_RAM_SIZE (128 * 1024)
/* AXI SRAM (all 320 KiB: less if option bytes give some to ITCM), SRAM1-2, SRAM4 */
#define MUBL_STACK_RAM {0x24000000, 0x24050000}, {0x30000000, 0x30008000}, {0x38000000, 0x38004000}
#define MUBL_H7_PWR_CR3 0x4                      /* SMPSEN: direct SMPS supply */

#define MUBL_USB 1
#define MUBL_LINK_PRIORITY MUBL_LINK_USB
#define MUBL_USB_OTG_INSTANCE 1                  /* USB1 OTG_HS, the only one on H72x/H73x */
#define MUBL_USB_DM_PIN MUBL_PIN('A', 11, 10)
#define MUBL_USB_DP_PIN MUBL_PIN('A', 12, 10)
#define MUBL_USB_PRODUCT "STM32H735G-DK bootloader"

#define MUBL_LED_PIN MUBL_PIN('C', 3, 0)         /* LD1, green */
#define MUBL_LED_ON 0
#define MUBL_HOLD_PIN MUBL_PIN('C', 13, 0)       /* B1 user button */
#define MUBL_HOLD_LEVEL 1
