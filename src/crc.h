#ifndef MUBL_CRC_H
#define MUBL_CRC_H

#include "mubl.h"

/* CRC-32 (reflected, 0xEDB88320) with no initial or final inversion: ArduPilot's crc32_small and
 * the PX4 bootloader protocol.  zlib's CRC-32 is ~crc32_update(~0U, ...).  In the CRC unit where
 * MUBL_HW_CRC32 (src/hal/stm32/crc_hw.c). */
uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n);

/* CRC-64-WE (0x42F0E1EBA9EA3693, MSB first).  Start from ~0 and invert the result. */
uint64_t crc64_update(uint64_t crc, const uint8_t* p, uint32_t n);

/* CRC-16 XMODEM (0x1021, MSB first, start 0): SRXL2 packets. */
uint16_t crc16_xmodem(uint16_t crc, const uint8_t* p, uint32_t n);

#endif
