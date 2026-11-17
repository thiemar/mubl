/*
 * Unsigned division for Cortex-M0, which has no divide instruction (nor the 32x32->64 multiply that
 * would let the compiler turn division by a constant into a multiply).  GCC calls these EABI helpers
 * for every `/` and `%`; defining them here keeps libgcc's speed-optimised versions (276 bytes) out.
 * µBL divides only at start-up, on a baud or bitrate change and once a second, so a plain
 * shift-and-subtract loop (32 iterations, a few microseconds at 48 MHz) is plenty.
 *
 * Division by zero returns a quotient of 0xFFFFFFFF and the dividend as remainder; µBL never
 * divides by zero.  Compiled to nothing on cores with a divide instruction.
 */
#include <stdint.h>

#if defined(__ARM_ARCH_6M__) || defined(MUBL_DIVIDE_TEST)

#if defined(__clang__)
#define KEEP __attribute__((used))
#else
/* externally_visible: with LTO the compiler emits these calls after deciding what is unused, so
 * they must stay global for the linker to prefer them to libgcc's. */
#define KEEP __attribute__((used, externally_visible))
#endif

/* Quotient in the low word (r0), remainder in the high word (r1), as the EABI requires. */
KEEP uint64_t __aeabi_uidivmod(uint32_t n, uint32_t d);
KEEP uint32_t __aeabi_uidiv(uint32_t n, uint32_t d);

uint64_t __aeabi_uidivmod(uint32_t n, uint32_t d)
{
    uint32_t q = 0, r = 0;
    /* Before each shift r holds fewer bits of n than have been taken (at most 31), so r << 1 never
     * overflows, whatever d is. */
    for (uint32_t bit = 1U << 31; bit != 0U; bit >>= 1) {
        r = (r << 1) | ((n & bit) ? 1U : 0U);
        if (r >= d) {
            r -= d;
            q |= bit;
        }
    }
    return ((uint64_t)r << 32) | q;
}

uint32_t __aeabi_uidiv(uint32_t n, uint32_t d)
{
    return (uint32_t)__aeabi_uidivmod(n, d);
}

#endif
