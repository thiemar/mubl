/*
 * Software CRCs.  The table versions take four bits per lookup (16-entry tables: small, and two to
 * three times faster than bit by bit); MUBL_CRC32_TABLE / MUBL_CRC64_TABLE 0 give the bitwise loops.
 * Where the CRC unit does CRC-32 (MUBL_HW_CRC32) it is in src/hal/stm32/crc_hw.c instead.
 */
#include "crc.h"

#if !MUBL_HW_CRC32
/* CRC-32, reflected polynomial 0xEDB88320. */
#if MUBL_CRC32_TABLE
static const uint32_t crc32_table[16] = {
    0x00000000U, 0x1DB71064U, 0x3B6E20C8U, 0x26D930ACU, 0x76DC4190U, 0x6B6B51F4U, 0x4DB26158U, 0x5005713CU,
    0xEDB88320U, 0xF00F9344U, 0xD6D6A3E8U, 0xCB61B38CU, 0x9B64C2B0U, 0x86D3D2D4U, 0xA00AE278U, 0xBDBDF21CU,
};

uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n)
{
    while (n--) {
        crc ^= *p++;
        crc = (crc >> 4) ^ crc32_table[crc & 15U];
        crc = (crc >> 4) ^ crc32_table[crc & 15U];
    }
    return crc;
}
#else
uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n)
{
    while (n--) {
        crc ^= *p++;
        for (uint32_t i = 8U; i != 0U; i--) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return crc;
}
#endif
#endif

#if MUBL_IMAGE_FORMATS & MUBL_IMG_LEGACY || MUBL_COMPAT_PX4_CAN
/* CRC-64-WE: polynomial 0x42F0E1EBA9EA3693, not reflected. */
#if MUBL_CRC64_TABLE
static const uint64_t crc64_table[16] = {
    0x0000000000000000ULL, 0x42F0E1EBA9EA3693ULL, 0x85E1C3D753D46D26ULL, 0xC711223CFA3E5BB5ULL,
    0x493366450E42ECDFULL, 0x0BC387AEA7A8DA4CULL, 0xCCD2A5925D9681F9ULL, 0x8E224479F47CB76AULL,
    0x9266CC8A1C85D9BEULL, 0xD0962D61B56FEF2DULL, 0x17870F5D4F51B498ULL, 0x5577EEB6E6BB820BULL,
    0xDB55AACF12C73561ULL, 0x99A54B24BB2D03F2ULL, 0x5EB4691841135847ULL, 0x1C4488F3E8F96ED4ULL,
};

uint64_t crc64_update(uint64_t crc, const uint8_t* p, uint32_t n)
{
    while (n--) {
        crc ^= (uint64_t)*p++ << 56;
        crc = (crc << 4) ^ crc64_table[crc >> 60];
        crc = (crc << 4) ^ crc64_table[crc >> 60];
    }
    return crc;
}
#else
uint64_t crc64_update(uint64_t crc, const uint8_t* p, uint32_t n)
{
    while (n--) {
        crc ^= (uint64_t)*p++ << 56;
        for (uint32_t i = 8U; i != 0U; i--) {
            crc = (crc & (1ULL << 63)) ? (crc << 1) ^ 0x42F0E1EBA9EA3693ULL : crc << 1;
        }
    }
    return crc;
}
#endif
#endif

#if MUBL_SRXL2
uint16_t crc16_xmodem(uint16_t crc, const uint8_t* p, uint32_t n)
{
    while (n--) {
        crc ^= (uint16_t)(*p++ << 8);
        for (uint8_t i = 0; i < 8U; i++) {
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
        }
    }
    return crc;
}
#endif
