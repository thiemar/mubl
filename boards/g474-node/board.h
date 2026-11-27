/*
 * STM32G474RE reference node: 512 KiB dual-bank flash (2 KiB pages, 256 KiB per bank), 128 KiB RAM,
 * 24 MHz crystal.  Exercises every STM32G4 peripheral µBL supports: USB FS (HSI48 + CRS), DroneCAN
 * over CAN FD (FDCAN1 on PB8/PB9 with 4 Mbps data phase), serial on USART2 (PA2/PA3) and SRXL2
 * single-wire on USART1 (PA9).
 */
#define MUBL_FAMILY MUBL_FAMILY_G4
#define MUBL_BOARD_NAME "org.sfwa.mubl.g474-node"
#define MUBL_BOARD_ID 6474

#define MUBL_BL_SIZE (20 * 1024)
#define MUBL_APP_SIZE (512 * 1024 - MUBL_BL_SIZE)
#define MUBL_G4_BANK_SIZE (256 * 1024)              /* dual-bank Cat.3: BKER from 0x08040000 */
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE (96 * 1024)                   /* SRAM1 (80 KiB) + SRAM2 (16 KiB) */
#define MUBL_STACK_RAM {0x10000000, 0x10008000}     /* 32 KiB CCM SRAM */
#define MUBL_HSE_HZ 24000000

#define MUBL_USB 1
#define MUBL_DRONECAN 1
#define MUBL_CAN_FD 1
#define MUBL_CAN_FD_DATA_BITRATE 4000000
#define MUBL_SERIAL 1
#define MUBL_SRXL2 1
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_DRONECAN, MUBL_LINK_SRXL2, MUBL_LINK_SERIAL
#define MUBL_SERIAL_CHIP_DES "STM32G474,0"

#define MUBL_CAN_INSTANCE 1
#define MUBL_CAN_RX_PIN MUBL_PIN('B', 8, 9)
#define MUBL_CAN_TX_PIN MUBL_PIN('B', 9, 9)

#define MUBL_UART0_INSTANCE 2
#define MUBL_UART0_TX_PIN MUBL_PIN('A', 2, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('A', 3, 7)
#define MUBL_SERIAL_PORT 0

#define MUBL_UART1_INSTANCE 1
#define MUBL_UART1_TX_PIN MUBL_PIN('A', 9, 7)       /* single wire */
#define MUBL_SRXL2_PORT 1
#define MUBL_SRXL2_DEVICE_ID 0x40
