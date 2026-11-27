/*
 * Mateksys mR24-30 2.4 GHz LoRa/FLRC radio module (STM32G431KBU6): 128 KiB flash (2 KiB pages),
 * 32 KiB RAM (SRAM1+SRAM2+CCM alias at 0x20000000), 8 MHz crystal, Semtech SX1281 + SKY65383-11.
 *
 * Flash layout (matching sfwa/airlink):
 *   0x08000000 - 0x08003FFF ( 16 KiB): µBL bootloader
 *   0x08004000 - 0x0801EFFF (108 KiB): application region
 *   0x0801F000 - 0x0801FFFF (  4 KiB): persistent application config (never erased by updates)
 *
 * Links:
 *   - USB FS CDC-ACM on USB-C / PA11-PA12 (HSI48 trimmed by CRS, internal D+ pull-up).
 *     MUBL_USB_STAY is enabled so the bootloader always stays whenever a USB host is connected
 *     until told to BOOT or disconnected.
 *   - Serial bootloader protocol on USART1 (PA9 TX / PA10 RX, AF7), the primary telemetry header.
 *
 * Safety outputs held while the bootloader runs: SX1281 CS high (PA4), PA TX_EN low (PB7),
 * LNA RX_EN low (PB0), cooling fan off (PA8).  Red LED on PA0 (active high); bind/boot button
 * on PB8 (active high).
 */
#define MUBL_FAMILY MUBL_FAMILY_G4
#define MUBL_BOARD_NAME "org.sfwa.mubl.matek-mr24-30"
#ifndef MUBL_BOARD_ID
#define MUBL_BOARD_ID 6431
#endif

#define MUBL_BL_SIZE (16 * 1024)
#define MUBL_APP_SIZE (128 * 1024 - MUBL_BL_SIZE - 4 * 1024)
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE (32 * 1024)
#define MUBL_STACK_RAM {0x10000000, 0x10002800}     /* CCM SRAM */
#define MUBL_HSE_HZ 8000000

#ifndef MUBL_USB
#define MUBL_USB 1
#endif
#ifndef MUBL_USB_STAY
#define MUBL_USB_STAY 1                             /* always stay while a USB host is connected */
#endif
#ifndef MUBL_BOOT_DELAY_MS
#define MUBL_BOOT_DELAY_MS 2000                     /* allow slow USB hubs to reset/enumerate before booting */
#endif
#ifndef MUBL_USB_GONE_MS
#define MUBL_USB_GONE_MS 1000
#endif
#ifndef MUBL_SERIAL
#define MUBL_SERIAL 1
#endif
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_SERIAL
#define MUBL_USB_PRODUCT "Matek mR24-30-BL"
#define MUBL_SERIAL_CHIP_DES "STM32G431,0"

#define MUBL_UART0_INSTANCE 1
#define MUBL_UART0_TX_PIN MUBL_PIN('A', 9, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('A', 10, 7)
#define MUBL_SERIAL_PORT 0

#define MUBL_LED_PIN MUBL_PIN('A', 0, 0)            /* red LED, active high */
#define MUBL_LED_ON 1
#define MUBL_HOLD_PIN MUBL_PIN('B', 8, 0)           /* button pulls PB8/BOOT0 high */
#define MUBL_HOLD_LEVEL 1
/* Keep SX1281 deselected (PA4 high) and PA/LNA/fan off (PB7, PB0, PA8 low) in the bootloader. */
#define MUBL_OUTPUT_PINS {MUBL_PIN('A', 4, 0), 1}, {MUBL_PIN('B', 7, 0), 0}, \
    {MUBL_PIN('B', 0, 0), 0}, {MUBL_PIN('A', 8, 0), 0}
