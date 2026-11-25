/*
 * STM32F042K6 DroneCAN-only node in 8 KiB: the f042-node hardware with the serial and SRXL2 links
 * left out, leaving 24 KiB for the application.
 */
#define MUBL_BL_SIZE (8 * 1024)
#define MUBL_DRONECAN 1
#define MUBL_SERIAL 0
#define MUBL_SRXL2 0
#define MUBL_LINK_PRIORITY MUBL_LINK_DRONECAN
#include "../f042-node/board.h"
#undef MUBL_BOARD_NAME
#define MUBL_BOARD_NAME "org.sfwa.mubl.f042-can"
