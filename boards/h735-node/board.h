/*
 * STM32H735 (RM0468): 1 MiB single-bank flash, direct SMPS supply, 25 MHz crystal, CAN FD.
 * DroneCAN over CAN FD (5 Mbit/s data phase) on FDCAN1 PD0/PD1; serial protocol on USART1
 * PA9/PA10.  Check the supply and pins against your board before flashing: a wrong MUBL_H7_PWR_CR3
 * can stop the part until power is removed.
 */
#define MUBL_FAMILY MUBL_FAMILY_H7
#define MUBL_BOARD_NAME "org.sfwa.mubl.h735"
#define MUBL_BOARD_ID 7351

#define MUBL_BL_SIZE (128 * 1024)
#define MUBL_APP_SIZE (1024 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000                 /* DTCM */
#define MUBL_RAM_SIZE (128 * 1024)
/* AXI SRAM (all 320 KiB: less if option bytes give some to ITCM), SRAM1-2, SRAM4 */
#define MUBL_STACK_RAM {0x24000000, 0x24050000}, {0x30000000, 0x30008000}, {0x38000000, 0x38004000}
#define MUBL_HSE_HZ 25000000
#define MUBL_H7_PWR_CR3 0x4                      /* SMPSEN: direct SMPS supply */

#define MUBL_DRONECAN 1
#define MUBL_SERIAL 1
#define MUBL_LINK_PRIORITY MUBL_LINK_DRONECAN, MUBL_LINK_SERIAL
#define MUBL_CAN_FD 1
#define MUBL_CAN_FD_DATA_BITRATE 5000000
#define MUBL_CAN_BITRATE 1000000

#define MUBL_CAN_RX_PIN MUBL_PIN('D', 0, 9)
#define MUBL_CAN_TX_PIN MUBL_PIN('D', 1, 9)

#define MUBL_UART0_INSTANCE 1
#define MUBL_UART0_TX_PIN MUBL_PIN('A', 9, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('A', 10, 7)
#define MUBL_SERIAL_PORT 0
