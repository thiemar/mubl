/* The bootloader board's settings (MUBL_TEST_BOARD, a path), with the test application's USB
 * identity: the host tells it from the bootloader by the product string. */
#include MUBL_TEST_BOARD
#undef MUBL_USB_PRODUCT
#define MUBL_USB_PRODUCT "mubl test app"
#define MUBL_USB_TX_SIZE 512
