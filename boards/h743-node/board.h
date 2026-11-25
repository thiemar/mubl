/*
 * STM32H743ZI (e.g. a NUCLEO-H743ZI): 2 MiB flash, LDO supply, 8 MHz HSE bypass (ST-LINK MCO).
 * DroneCAN on FDCAN1 PD0/PD1; serial protocol on USART3 PD8/PD9 (the Nucleo's virtual COM port);
 * SRXL2 single-wire on USART2 PD5; USB on the Nucleo's user connector (OTG_FS, PA11/PA12).  The bootloader is sector 0 (128 KiB).
 */
#define MUBL_FAMILY MUBL_FAMILY_H7
#define MUBL_BOARD_NAME "org.sfwa.mubl.h743"
#define MUBL_BOARD_ID 7431

#define MUBL_BL_SIZE (128 * 1024)
#define MUBL_APP_SIZE (2048 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000                 /* DTCM */
#define MUBL_RAM_SIZE (128 * 1024)
/* AXI SRAM, SRAM1-3, SRAM4 */
#define MUBL_STACK_RAM {0x24000000, 0x24080000}, {0x30000000, 0x30048000}, {0x38000000, 0x38010000}
#define MUBL_HSE_HZ 8000000
#define MUBL_HSE_BYPASS 1
#define MUBL_H7_PWR_CR3 0x2                      /* LDOEN, SCUEN cleared: LDO supply, locked */
#define MUBL_H7_SRAM3 1                          /* H74x: SRAM1-3 in the D2 domain */

#define MUBL_DRONECAN 1
#define MUBL_SERIAL 1
#define MUBL_SRXL2 1
#ifndef MUBL_USB
#define MUBL_USB 1
#endif
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_DRONECAN, MUBL_LINK_SERIAL, MUBL_LINK_SRXL2
#define MUBL_USB_OTG_INSTANCE 2                  /* USB2 OTG_FS */
#define MUBL_USB_DM_PIN MUBL_PIN('A', 11, 10)
#define MUBL_USB_DP_PIN MUBL_PIN('A', 12, 10)

#define MUBL_CAN_RX_PIN MUBL_PIN('D', 0, 9)
#define MUBL_CAN_TX_PIN MUBL_PIN('D', 1, 9)

#define MUBL_UART0_INSTANCE 3
#define MUBL_UART0_TX_PIN MUBL_PIN('D', 8, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('D', 9, 7)
#define MUBL_SERIAL_PORT 0

#define MUBL_UART1_INSTANCE 2
#define MUBL_UART1_TX_PIN MUBL_PIN('D', 5, 7)    /* single wire */
#define MUBL_SRXL2_PORT 1

#define MUBL_LED_PIN MUBL_PIN('B', 0, 0)         /* LD1 */
#define MUBL_HOLD_PIN MUBL_PIN('C', 13, 0)       /* B1, active high */
#define MUBL_HOLD_LEVEL 1
