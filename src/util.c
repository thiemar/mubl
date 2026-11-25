/*
 * The few C library functions the compiler may call, so the bootloader links without a C library.
 * The host simulation uses the system's.
 */
#include "mubl.h"

#ifndef MUBL_SIM
/* `used`: with LTO the compiler may introduce calls (struct copies) after deciding these are dead. */
__attribute__((used)) void* memcpy(void* dst, const void* src, size_t n)
{
    uint8_t* d = dst;
    const uint8_t* s = src;
    while (n--) *d++ = *s++;
    return dst;
}

__attribute__((used)) void* memmove(void* dst, const void* src, size_t n)
{
    uint8_t* d = dst;
    const uint8_t* s = src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        while (n--) d[n] = s[n];
    }
    return dst;
}

__attribute__((used)) void* memset(void* dst, int c, size_t n)
{
    uint8_t* d = dst;
    while (n--) *d++ = (uint8_t)c;
    return dst;
}

__attribute__((used)) int memcmp(const void* a, const void* b, size_t n)
{
    const uint8_t* x = a;
    const uint8_t* y = b;
    for (; n; n--, x++, y++) {
        if (*x != *y) return *x - *y;
    }
    return 0;
}
#endif
