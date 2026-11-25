/*
 * Forced into the sanitizer builds (tests/test_fuzz.py): glibc declares the pointer arguments of
 * the mem* functions nonnull, so UBSan reports a null pointer passed to them even with a length of
 * zero; macOS's headers do not, and the same code passed there.  Declaring them as glibc does makes
 * every host report it.  (Built with _FORTIFY_SOURCE off, whose macros would bypass these.)
 */
#include <stddef.h>

void* memcpy(void* restrict, const void* restrict, size_t) __attribute__((nonnull(1, 2)));
void* memmove(void*, const void*, size_t) __attribute__((nonnull(1, 2)));
void* memset(void*, int, size_t) __attribute__((nonnull(1)));
int memcmp(const void*, const void*, size_t) __attribute__((nonnull(1, 2)));
