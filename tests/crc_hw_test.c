/*
 * src/hal/stm32/crc_hw.c against the model of the STM32 CRC unit (src/hal/host/crc_model.c).  The
 * driver must give the same answers as the reflected software CRC-32.
 */
#include <stdio.h>
#include <stdlib.h>

#include "crc.h"
#include "hal/host/crc_model.h"

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
    crc_model_clock(true);
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
    if (crc_model_errors != 0U) {
        printf("CRC unit misused %u times\n", crc_model_errors);
        bad++;
    }
    printf(bad ? "FAILED\n" : "ok\n");
    return bad != 0;
}
