/*
 * Host model of the STM32 CRC unit (F0/F3/H7, as RM0091, RM0316 and RM0433 describe it), in place
 * of its registers for src/hal/stm32/crc_hw.c built with MUBL_CRC_MODEL: polynomial 0x04C11DB7
 * computed MSB first; REV_IN = 01 reverses the bits of each byte written, in place; REV_OUT reverses
 * the 32-bit result; RESET loads INIT.
 *
 * Its clock follows the HAL, as on STM32: off after a reset, started by hal_init, stopped by
 * hal_deinit.  With the clock off writes are ignored and reads return 0, so a CRC taken then comes
 * out wrong as on the hardware; such uses, and an unexpected CR, are counted in crc_model_errors.
 */
#include "hal/host/crc_model.h"

static uint32_t reg, cr;
static bool clock_on;
unsigned crc_model_errors;

void crc_model_clock(bool on) { clock_on = on; }

static uint32_t rev32(uint32_t x)
{
    uint32_t r = 0;
    for (int i = 0; i < 32; i++) r |= ((x >> i) & 1U) << (31 - i);
    return r;
}

static uint32_t rev_bytes(uint32_t w)        /* REV_IN = 01: each byte's bits, in place */
{
    uint32_t r = 0;
    for (int b = 0; b < 4; b++) r |= (rev32((w >> (8 * b)) & 0xFFU) >> 24) << (8 * b);
    return r;
}

static bool clocked(void)
{
    if (!clock_on) crc_model_errors++;
    return clock_on;
}

static void feed(uint32_t data, int bits)
{
    if (!clocked()) return;
    reg ^= bits == 32 ? data : data << (32 - bits);
    for (int i = 0; i < bits; i++) reg = (reg & 0x80000000U) ? (reg << 1) ^ 0x04C11DB7U : reg << 1;
}

void model_start(uint32_t init, uint32_t c)
{
    if (!clocked()) return;
    if ((c & 0x60U) != 0x20U || !(c & 0x80U)) crc_model_errors++;
    cr = c;
    reg = init;                                /* RESET: DR = INIT */
}

void model_put8(uint8_t b) { feed(rev_bytes(b), 8); }
void model_put32(uint32_t w) { feed(rev_bytes(w), 32); }
uint32_t model_get(void) { return !clocked() ? 0U : (cr & 0x80U) ? rev32(reg) : reg; }
