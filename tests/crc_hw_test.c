/*
 * src/hal/stm32/crc_hw.c against a software model of the STM32 CRC unit (F0/F3/H7, as RM0091,
 * RM0316 and RM0433 describe it): polynomial 0x04C11DB7 computed MSB first; REV_IN = 01 reverses
 * the bits of each byte of what is written, in place; REV_OUT reverses the 32-bit result; RESET
 * loads INIT.  The driver must give the same answers as the reflected software CRC-32.
 */
#include <stdio.h>
#include <stdlib.h>

#include "crc.h"

static uint32_t reg, cr;

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

static void feed(uint32_t data, int bits)
{
    reg ^= bits == 32 ? data : data << (32 - bits);
    for (int i = 0; i < bits; i++) reg = (reg & 0x80000000U) ? (reg << 1) ^ 0x04C11DB7U : reg << 1;
}

void model_start(uint32_t init, uint32_t c)
{
    if ((c & 0x60U) != 0x20U || !(c & 0x80U)) {
        printf("unexpected CR %08x\n", c);
        exit(1);
    }
    cr = c;
    reg = init;                                /* RESET: DR = INIT */
}
void model_put8(uint8_t b) { feed(rev_bytes(b), 8); }
void model_put32(uint32_t w) { feed(rev_bytes(w), 32); }
uint32_t model_get(void) { return (cr & 0x80U) ? rev32(reg) : reg; }

static uint32_t reference(uint32_t crc, const uint8_t* p, uint32_t n)   /* crc32_small */
{
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return crc;
}

int main(void)
{
    static uint8_t buf[512];
    uint32_t x = 99;
    int bad = 0;
    if ((~crc32_update(~0U, (const uint8_t*)"123456789", 9)) != 0xCBF43926U) {
        printf("zlib check value wrong\n");
        bad++;
    }
    for (int i = 0; i < 200000 && bad < 10; i++) {
        x ^= x << 13, x ^= x >> 17, x ^= x << 5;
        const uint32_t off = x & 7U, len = (x >> 3) % 300U, init = (x & 0x100U) ? x : (x & 0x200U ? ~0U : 0U);
        for (uint32_t k = 0; k < off + len; k++) buf[k] = (uint8_t)(x >> (k & 15)) ^ (uint8_t)k;
        const uint32_t got = crc32_update(init, buf + off, len), want = reference(init, buf + off, len);
        if (got != want) {
            printf("offset %u length %u init %08x: %08x, expected %08x\n", off, len, init, got, want);
            bad++;
        }
    }
    printf(bad ? "FAILED\n" : "ok\n");
    return bad != 0;
}
