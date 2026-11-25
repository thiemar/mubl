/*
 * CRC-32 in the CRC unit (F3, H7 by default; F0 can too, with MUBL_HW_CRC32), the same function as the software crc32_update: reflected
 * polynomial 0xEDB88320, no inversion, continuing from `crc`.
 *
 * The unit computes MSB first with polynomial 0x04C11DB7 (its reset value; F04x has no other).
 * A reflected CRC is the same computation on bit-reversed data with a bit-reversed register, so:
 * INIT = reverse(crc), input bits reversed within each byte (REV_IN = 01), output reversed
 * (REV_OUT).  Bytes are fed one at a time until the pointer is word aligned, then as words; a
 * little-endian word is byte-swapped first so its lowest-addressed byte is processed first, as in
 * the byte stream.  Writes stall the bus until the unit is ready, so no waiting is needed.
 */
#include "hal/hal.h"
#include "crc.h"

#if MUBL_HW_CRC32

#ifdef MUBL_CRC_MODEL
/* tests/crc_hw_test.c: a software model of the peripheral in place of its registers. */
void model_start(uint32_t init, uint32_t cr);
void model_put8(uint8_t b);
void model_put32(uint32_t w);
uint32_t model_get(void);
#define CRC_CR_REV_IN_BYTE (1U << 5)                 /* as in stm32.h */
#define CRC_CR_REV_OUT (1U << 7)
#define CRC_START(init, cr) model_start((init), (cr))
#define CRC_PUT8(b) model_put8(b)
#define CRC_PUT32(w) model_put32(w)
#define CRC_GET() model_get()
#else
#include "stm32.h"
#define CRC_START(init, cr) (REG32(STM32_CRC + CRC_INIT) = (init), REG32(STM32_CRC + CRC_CR) = (cr) | CRC_CR_RESET)
#define CRC_PUT8(b) (*(volatile uint8_t*)(uintptr_t)(STM32_CRC + CRC_DR) = (b))
#define CRC_PUT32(w) (REG32(STM32_CRC + CRC_DR) = (w))
#define CRC_GET() REG32(STM32_CRC + CRC_DR)
#endif

static uint32_t reverse(uint32_t x)
{
    x = ((x >> 1) & 0x55555555U) | ((x & 0x55555555U) << 1);
    x = ((x >> 2) & 0x33333333U) | ((x & 0x33333333U) << 2);
    x = ((x >> 4) & 0x0F0F0F0FU) | ((x & 0x0F0F0F0FU) << 4);
    return __builtin_bswap32(x);
}

uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n)
{
    CRC_START(reverse(crc), CRC_CR_REV_IN_BYTE | CRC_CR_REV_OUT);
    for (; n != 0U && ((uintptr_t)p & 3U) != 0U; n--) CRC_PUT8(*p++);
    for (; n >= 4U; n -= 4U, p += 4) CRC_PUT32(__builtin_bswap32(*(const uint32_t*)(const void*)p));
    for (; n != 0U; n--) CRC_PUT8(*p++);
    return CRC_GET();
}

#endif
