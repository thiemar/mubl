/* src/hal/stm32/divide.c on the host, against the compiler's division. */
#define MUBL_DIVIDE_TEST 1
#include "../src/hal/stm32/divide.c"

#include <stdio.h>

static int check(uint32_t n, uint32_t d)
{
    const uint64_t qr = __aeabi_uidivmod(n, d);
    const uint32_t q = (uint32_t)qr, r = (uint32_t)(qr >> 32);
    const uint32_t eq = d ? n / d : 0xFFFFFFFFU, er = d ? n % d : n;
    if (q != eq || r != er || __aeabi_uidiv(n, d) != eq) {
        printf("%u / %u: got %u r %u, expected %u r %u\n", n, d, q, r, eq, er);
        return 1;
    }
    return 0;
}

int main(void)
{
    static const uint32_t edge[] = {0, 1, 2, 3, 7, 999, 1000, 1001, 115200, 48000000U, 0x7FFFFFFFU,
                                    0x80000000U, 0x80000001U, 0xFFFFFFFEU, 0xFFFFFFFFU};
    const unsigned ne = sizeof edge / sizeof edge[0];
    int bad = 0;
    for (unsigned i = 0; i < ne; i++) {
        for (unsigned j = 0; j < ne; j++) bad += check(edge[i], edge[j]);
    }
    uint32_t x = 0x12345678U;
    for (unsigned i = 0; i < 1000000U; i++) {
        x ^= x << 13, x ^= x >> 17, x ^= x << 5;
        const uint32_t n = x;
        x ^= x << 13, x ^= x >> 17, x ^= x << 5;
        const uint32_t d = x >> (x & 31U);           /* divisors of every magnitude */
        bad += check(n, d);
        if (bad > 10) break;
    }
    printf(bad ? "FAILED\n" : "ok\n");
    return bad != 0;
}
