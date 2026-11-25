/*
 * STM32F303CC with USB: 256 KiB flash, 40 KiB RAM, 8 MHz crystal (USB needs it: PLL 72 MHz / 1.5).
 * USB on PA11/PA12 with a fixed 1.5 kOhm D+ pull-up (D+ is driven low to detach); serial protocol on
 * USART1 (PA9/PA10); SRXL2 single-wire on USART2 (PA2).  No DroneCAN: on the F303xB/C, USB and
 * CAN share their packet memory and cannot run together (f303-node has CAN on these pins).
 */
#define MUBL_FAMILY MUBL_FAMILY_F3
#define MUBL_BOARD_NAME "org.sfwa.mubl.f303-usb"
#define MUBL_BOARD_ID 4302

#define MUBL_BL_SIZE (16 * 1024)
#define MUBL_APP_SIZE (256 * 1024 - MUBL_BL_SIZE)
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE (40 * 1024)
#define MUBL_STACK_RAM {0x10000000, 0x10002000}     /* CCM */
#define MUBL_HSE_HZ 8000000

#define MUBL_USB 1
#define MUBL_SERIAL 1
#define MUBL_SRXL2 1
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_SRXL2, MUBL_LINK_SERIAL

#define MUBL_USB_DM_PIN MUBL_PIN('A', 11, 14)
#define MUBL_USB_DP_PIN MUBL_PIN('A', 12, 14)

#define MUBL_UART0_INSTANCE 1
#define MUBL_UART0_TX_PIN MUBL_PIN('A', 9, 7)
#define MUBL_UART0_RX_PIN MUBL_PIN('A', 10, 7)
#define MUBL_SERIAL_PORT 0

#define MUBL_UART1_INSTANCE 2
#define MUBL_UART1_TX_PIN MUBL_PIN('A', 2, 7)    /* single wire */
#define MUBL_SRXL2_PORT 1
#define MUBL_SRXL2_DEVICE_ID 0x40
