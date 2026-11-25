/*
 * STM32F042C6 USB-CAN adapter (µCAN hardware): 32 KiB flash, 6 KiB RAM, no crystal.  USB on
 * PA11/PA12 (HSI48, trimmed by the CRS against the host's start-of-frame packets).
 *
 * USB only, in 8 KiB: the 24 KiB left holds µCAN's application (23 KiB).  With DroneCAN as well
 * (CAN on PB8/PB9: -DMUBL_DRONECAN=1, 13 KiB) note that without a crystal HSI48 is accurate enough
 * for CAN only once the CRS has locked to a USB host.
 */
#define MUBL_FAMILY MUBL_FAMILY_F0
#define MUBL_BOARD_NAME "org.sfwa.mubl.f042-usb"
#define MUBL_BOARD_ID 4202

#ifndef MUBL_DRONECAN
#define MUBL_DRONECAN 0
#endif
#ifndef MUBL_BL_SIZE
#define MUBL_BL_SIZE ((MUBL_DRONECAN ? 13 : 8) * 1024)
#endif
#define MUBL_APP_SIZE (32 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE (6 * 1024)
#define MUBL_MIN_STACK 768
#define MUBL_F0_HSI48 1

#define MUBL_USB 1
#if MUBL_DRONECAN
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_DRONECAN
#else
#define MUBL_LINK_PRIORITY MUBL_LINK_USB
#endif
#define MUBL_DC_LOG 0

#define MUBL_CAN_RX_PIN MUBL_PIN('B', 8, 4)
#define MUBL_CAN_TX_PIN MUBL_PIN('B', 9, 4)

#define MUBL_USB_MANUFACTURER "Thiemar"
#define MUBL_USB_PRODUCT "muCAN bootloader"
