/*
 * TBS Lucid H7 flight controller (STM32H743): 2 MiB flash, LDO supply, 8 MHz crystal.  Links:
 *
 * - USB on the board's USB connector (OTG_FS on PA11/PA12, no VBUS sensing; HSI48 trimmed by the
 *   CRS from the host's start-of-frame packets).
 * - DroneCAN on the CAN connector (FDCAN1 on PD0/PD1, kernel clock from the crystal); PD3 is held
 *   low to keep the transceiver out of silent mode.
 * - The serial protocol on UART7 (pads R7/T7: PE7 RX, PE8 TX; no flow control), the telemetry port
 *   ArduPilot's bootloader listens on (SERIAL7, Telem2, in ArduPilot).
 *
 * Each can be left out: -DMUBL_DRONECAN=0, -DMUBL_SERIAL=0 (both for the USB-only build).  Set up as
 * ArduPilot's bootloader for the board (libraries/AP_HAL_ChibiOS/hwdef/TBS_LUCID_H7/hwdef-bl.dat),
 * so ArduPilot's .apj files load with its uploader.py:
 *
 * - The bootloader is sector 0 and the application starts at 128 KiB.  The top two sectors (256 KiB)
 *   hold ArduPilot's parameters (STORAGE_FLASH_PAGE 14) and are left out of the application area, so
 *   an update never erases them; 1664 KiB is also what ArduPilot's bootloader reports.
 * - The board ID is ArduPilot's AP_HW_TBS_LUCID_H7.  The variants have the same pins: build them
 *   with EXTRA_CFLAGS=-DMUBL_BOARD_ID=5253 (H7 Wing), 5254 (H7 Wing AIO), 5255 (H7 OEM) or 5256
 *   (H7 V3).  On the OEM and V3, PB12 is UART5's RX and is not driven.
 * - While the bootloader runs, the 9 V VTX supply (PC13), the VSW pads (PD10) and the camera
 *   switch (PD11) are low (supplies off, camera 2 selected) and the SPI chip selects are high.
 * - The blue LED (PE3, active low) is on in the bootloader.  There is no button to hold it: the
 *   board's BOOT button is BOOT0, which starts ST's ROM DFU loader instead.
 * - Images without a descriptor boot too (MUBL_RAW_IMAGES): ArduPilot puts one only in AP_Periph
 *   and Remote ID builds, and Betaflight never does.  Such an image is checked only for a plausible
 *   stack pointer and reset vector; an interrupted update still leaves it unbootable, as its first
 *   block is written last.  Images that have a descriptor are still checked by it (CRCs, board ID).
 *   -DMUBL_RAW_IMAGES=0 boots only images with a descriptor.
 * - The USB product string ends in "-BL", as ArduPilot's bootloaders do, so that uploader.py's
 *   default port list finds it on Linux (/dev/serial/by-id/usb-*-BL_*).
 * - While a USB host is connected the bootloader stays (MUBL_USB_STAY), also after the application
 *   reboots, until uploader.py sends BOOT or the cable is pulled; ArduPilot's own bootloader instead
 *   boots after its delay.  EXTRA_CFLAGS=-DMUBL_USB_STAY=0 gives that behaviour.
 */
#define MUBL_FAMILY MUBL_FAMILY_H7
#define MUBL_BOARD_NAME "org.sfwa.mubl.tbs-lucid-h7"
#ifndef MUBL_BOARD_ID
#define MUBL_BOARD_ID 5250                       /* AP_HW_TBS_LUCID_H7 */
#endif

#define MUBL_BL_SIZE (128 * 1024)
#define MUBL_APP_SIZE (2048 * 1024 - MUBL_BL_SIZE - 256 * 1024)
#define MUBL_RAM_BASE 0x20000000                 /* DTCM */
#define MUBL_RAM_SIZE (128 * 1024)
/* AXI SRAM, SRAM1-3, SRAM4 */
#define MUBL_STACK_RAM {0x24000000, 0x24080000}, {0x30000000, 0x30048000}, {0x38000000, 0x38010000}
#define MUBL_H7_PWR_CR3 0x2                      /* LDOEN, SCUEN cleared: LDO supply, locked */
#define MUBL_H7_SRAM3 1                          /* H74x: SRAM1-3 in the D2 domain */
#ifndef MUBL_RAW_IMAGES
#define MUBL_RAW_IMAGES 1                        /* stock ArduPilot and Betaflight have no descriptor */
#endif

#define MUBL_HSE_HZ 8000000

#define MUBL_USB 1
#ifndef MUBL_DRONECAN
#define MUBL_DRONECAN 1
#endif
#ifndef MUBL_SERIAL
#define MUBL_SERIAL 1
#endif
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_DRONECAN, MUBL_LINK_SERIAL
#define MUBL_USB_OTG_INSTANCE 2                  /* USB2 OTG_FS */
#define MUBL_USB_DM_PIN MUBL_PIN('A', 11, 10)
#define MUBL_USB_DP_PIN MUBL_PIN('A', 12, 10)
#define MUBL_USB_PRODUCT "TBS Lucid H7-BL"

#define MUBL_CAN_RX_PIN MUBL_PIN('D', 0, 9)
#define MUBL_CAN_TX_PIN MUBL_PIN('D', 1, 9)
#define MUBL_CAN_STANDBY_PIN MUBL_PIN('D', 3, 0)  /* transceiver silent when high */
#define MUBL_CAN_STANDBY_LEVEL 1

#define MUBL_UART0_INSTANCE 7
#define MUBL_UART0_TX_PIN MUBL_PIN('E', 8, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('E', 7, 7)
#define MUBL_SERIAL_PORT 0

#define MUBL_LED_PIN MUBL_PIN('E', 3, 0)         /* LED0, blue */
#define MUBL_LED_ON 0

#define LUCID_POWER_OFF {MUBL_PIN('C', 13, 0), 0}, {MUBL_PIN('D', 10, 0), 0}, {MUBL_PIN('D', 11, 0), 0}
/* IMU1, IMU2, external 1 and 2 */
#define LUCID_CS_HIGH {MUBL_PIN('C', 15, 0), 1}, {MUBL_PIN('E', 11, 0), 1}, {MUBL_PIN('D', 4, 0), 1}, \
    {MUBL_PIN('E', 2, 0), 1}
#if MUBL_BOARD_ID == 5255 || MUBL_BOARD_ID == 5256
#define MUBL_OUTPUT_PINS LUCID_POWER_OFF, LUCID_CS_HIGH
#else
#define MUBL_OUTPUT_PINS LUCID_POWER_OFF, LUCID_CS_HIGH, {MUBL_PIN('B', 12, 0), 1}   /* MAX7456 OSD */
#endif
