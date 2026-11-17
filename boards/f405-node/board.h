/*
 * STM32F405RG node: 1 MiB flash, 128 KiB RAM, 8 MHz crystal, 168 MHz.
 * DroneCAN on CAN1 PB8/PB9; serial protocol on USART1 (PA9/PA10); SRXL2 single-wire on USART3
 * (PB10).  The bootloader is sector 0 (16 KiB); the application starts at sector 1.
 */
#define MUBL_FAMILY MUBL_FAMILY_F4
#define MUBL_BOARD_NAME "org.sfwa.mubl.f405"
#define MUBL_BOARD_ID 4501

#define MUBL_BL_SIZE (16 * 1024)
#define MUBL_APP_SIZE (1024 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE (128 * 1024)
#define MUBL_STACK_RAM {0x10000000, 0x10010000}     /* CCM */
#define MUBL_HSE_HZ 8000000
#define MUBL_SYSCLK_HZ 168000000

#define MUBL_DRONECAN 1
#define MUBL_SERIAL 1
#define MUBL_SRXL2 1
#define MUBL_LINK_PRIORITY MUBL_LINK_SERIAL, MUBL_LINK_DRONECAN, MUBL_LINK_SRXL2
#define MUBL_COMPAT_AP_PERIPH 1
#define MUBL_COMPAT_PX4_CAN 1

#define MUBL_CAN_RX_PIN MUBL_PIN('B', 8, 9)
#define MUBL_CAN_TX_PIN MUBL_PIN('B', 9, 9)

#define MUBL_UART0_INSTANCE 1
#define MUBL_UART0_TX_PIN MUBL_PIN('A', 9, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('A', 10, 7)
#define MUBL_SERIAL_PORT 0

#define MUBL_UART1_INSTANCE 3
#define MUBL_UART1_TX_PIN MUBL_PIN('B', 10, 7)   /* single wire */
#define MUBL_SRXL2_PORT 1
#define MUBL_SRXL2_DEVICE_ID 0x30

#define MUBL_LED_PIN MUBL_PIN('C', 13, 0)
#define MUBL_LED_ON 0
