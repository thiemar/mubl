/*
 * STM32F042K6 node: 32 KiB flash, 6 KiB RAM, 8 MHz crystal.
 * DroneCAN on PA11/PA12; one single-wire USART2 on PA2 shared by SRXL2 and the serial protocol.
 */
#define MUBL_FAMILY MUBL_FAMILY_F0
#define MUBL_BOARD_NAME "org.sfwa.mubl.f042"
#define MUBL_BOARD_ID 4201

#ifndef MUBL_BL_SIZE
#define MUBL_BL_SIZE (12 * 1024)
#endif
#define MUBL_APP_SIZE (32 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE (6 * 1024)
#define MUBL_MIN_STACK 768
#define MUBL_HSE_HZ 8000000

#ifndef MUBL_DRONECAN
#define MUBL_DRONECAN 1
#endif
#ifndef MUBL_SERIAL
#define MUBL_SERIAL 1
#endif
#ifndef MUBL_SRXL2
#define MUBL_SRXL2 1
#endif
#ifndef MUBL_LINK_PRIORITY
#define MUBL_LINK_PRIORITY MUBL_LINK_DRONECAN, MUBL_LINK_SRXL2, MUBL_LINK_SERIAL
#endif
#define MUBL_DC_LOG 0            /* 520 B on Cortex-M0; NodeStatus still reports the outcome */

#define MUBL_CAN_RX_PIN MUBL_PIN('A', 11, 4)
#define MUBL_CAN_TX_PIN MUBL_PIN('A', 12, 4)

#if MUBL_SERIAL || MUBL_SRXL2
#define MUBL_UART0_INSTANCE 2
#define MUBL_UART0_TX_PIN MUBL_PIN('A', 2, 1)   /* no RX pin: single wire */
#define MUBL_SERIAL_PORT 0
#define MUBL_SRXL2_PORT 0
#define MUBL_SRXL2_DEVICE_ID 0x40
#endif
