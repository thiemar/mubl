/*
 * Flash erase and program, per family.  Every operation unlocks the controller, clears stale
 * status, runs, checks the error flags and locks again.  The update engine reads back and compares
 * everything it programs.
 *
 *   F0/F3: pages (MUBL_FLASH_PAGE), half-word programming.
 *   F4:    sectors of 16/16/16/16/64/128... KiB per 1 MiB bank (bank 2 from 0x08100000 on 2 MiB
 *          parts), word programming at MUBL_F4_PSIZE; the ART data cache is flushed afterwards.
 *   H7:    128 KiB sectors, one bank per MiB, 256-bit flash words written as eight 32-bit stores.
 */
#include "hal/hal.h"
#include "stm32.h"

MUBL_STATIC_ASSERT(MUBL_APP_BASE % 4 == 0, "application base must be word aligned");

#if MUBL_FAMILY == MUBL_FAMILY_F0 || MUBL_FAMILY == MUBL_FAMILY_F3

MUBL_STATIC_ASSERT(MUBL_FLASH_GRANULE == 2, "F0/F3 program half-words");
MUBL_STATIC_ASSERT(MUBL_APP_BASE % MUBL_FLASH_PAGE == 0 && MUBL_APP_SIZE % MUBL_FLASH_PAGE == 0,
                   "the application region must be whole pages");

static bool finish(void)
{
    while (FLASH_SR & FLASH_SR_BSY) {
    }
    const bool ok = (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) == 0U;
    FLASH_SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
    FLASH_CR = FLASH_CR_LOCK;
    return ok;
}

static void unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
    while (FLASH_SR & FLASH_SR_BSY) {
    }
    FLASH_SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
}

void hal_flash_sector(uint32_t addr, uint32_t* start, uint32_t* end)
{
    *start = addr & ~(MUBL_FLASH_PAGE - 1U);
    *end = *start + MUBL_FLASH_PAGE;
}

bool hal_flash_erase(uint32_t start)
{
    if (start < MUBL_APP_BASE) return false;
    unlock();
    FLASH_CR = FLASH_CR_PER;
    FLASH_AR = start;
    FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT;
    return finish();
}

bool hal_flash_program(uint32_t addr, const uint8_t* data)
{
    if (addr < MUBL_APP_BASE) return false;
    unlock();
    FLASH_CR = FLASH_CR_PG;
    *(volatile uint16_t*)(uintptr_t)addr = (uint16_t)(data[0] | (data[1] << 8));
    return finish();
}

#elif MUBL_FAMILY == MUBL_FAMILY_F4

MUBL_STATIC_ASSERT(MUBL_FLASH_GRANULE == 4 && MUBL_F4_PSIZE == 2, "F4 programs words (x32)");

/* Sector number of an offset in flash, and its bounds. */
static uint32_t sector(uint32_t off, uint32_t* start, uint32_t* end)
{
    const uint32_t bank = off >= 0x100000U ? 1U : 0U, rel = off & 0xFFFFFU;
    uint32_t n, s, size;
    if (rel < 0x10000U) {
        n = rel / 0x4000U;
        s = n * 0x4000U;
        size = 0x4000U;
    } else if (rel < 0x20000U) {
        n = 4U;
        s = 0x10000U;
        size = 0x10000U;
    } else {
        n = 4U + rel / 0x20000U;
        s = (n - 4U) * 0x20000U;
        size = 0x20000U;
    }
    *start = MUBL_FLASH_BASE + bank * 0x100000U + s;
    *end = *start + size;
    return bank ? (0x10U | n) : n;                  /* SNB: bank 2 sectors 12-23 are 0b1xxxx */
}

static void flush_dcache(void)
{
    const uint32_t acr = FLASH_ACR;
    FLASH_ACR = acr & ~FLASH_ACR_DCEN;
    FLASH_ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
    FLASH_ACR = acr;
}

static void unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
    while (FLASH_SR & FLASH_SR_BSY) {
    }
    FLASH_SR = FLASH_SR_ERRORS | 1U;                /* and EOP */
}

static bool finish(void)
{
    while (FLASH_SR & FLASH_SR_BSY) {
    }
    const bool ok = (FLASH_SR & FLASH_SR_ERRORS) == 0U;
    FLASH_SR = FLASH_SR_ERRORS | 1U;
    FLASH_CR = FLASH_CR_LOCK;
    flush_dcache();
    return ok;
}

void hal_flash_sector(uint32_t addr, uint32_t* start, uint32_t* end)
{
    (void)sector(addr - MUBL_FLASH_BASE, start, end);
}

bool hal_flash_erase(uint32_t start)
{
    uint32_t s, e;
    const uint32_t snb = sector(start - MUBL_FLASH_BASE, &s, &e);
    if (start < MUBL_APP_BASE || s != start) return false;
    unlock();
    FLASH_CR = FLASH_CR_SER | (snb << FLASH_CR_SNB_POS) | (MUBL_F4_PSIZE << FLASH_CR_PSIZE_POS);
    FLASH_CR |= FLASH_CR_STRT;
    return finish();
}

bool hal_flash_program(uint32_t addr, const uint8_t* data)
{
    if (addr < MUBL_APP_BASE) return false;
    unlock();
    FLASH_CR = FLASH_CR_PG | (MUBL_F4_PSIZE << FLASH_CR_PSIZE_POS);
    *(volatile uint32_t*)(uintptr_t)addr = mubl_get32(data);
    return finish();
}

#elif MUBL_FAMILY == MUBL_FAMILY_G4

MUBL_STATIC_ASSERT(MUBL_FLASH_GRANULE == 8, "G4 programs 64-bit double words");
MUBL_STATIC_ASSERT(MUBL_APP_BASE % MUBL_FLASH_PAGE == 0 && MUBL_APP_SIZE % MUBL_FLASH_PAGE == 0,
                   "the application region must be whole pages");

static void flush_caches(void)
{
    const uint32_t acr = FLASH_ACR;
    FLASH_ACR = acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH_ACR = (acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH_ACR = acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH_ACR = acr;
}

static void unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
    while (FLASH_SR & FLASH_SR_BSY) {
    }
    FLASH_SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
}

static bool finish(void)
{
    while (FLASH_SR & FLASH_SR_BSY) {
    }
    const bool ok = (FLASH_SR & FLASH_SR_ERRORS) == 0U;
    FLASH_SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    FLASH_CR = FLASH_CR_LOCK;
    flush_caches();
    return ok;
}

void hal_flash_sector(uint32_t addr, uint32_t* start, uint32_t* end)
{
    *start = addr & ~(MUBL_FLASH_PAGE - 1U);
    *end = *start + MUBL_FLASH_PAGE;
}

bool hal_flash_erase(uint32_t start)
{
    if (start < MUBL_APP_BASE || start % MUBL_FLASH_PAGE != 0U) return false;
    const uint32_t off = start - MUBL_FLASH_BASE;
    uint32_t bker = 0U, pnb = off / MUBL_FLASH_PAGE;
#if MUBL_G4_BANK_SIZE
    if (off >= MUBL_G4_BANK_SIZE) {
        bker = FLASH_CR_BKER;
        pnb = (off - MUBL_G4_BANK_SIZE) / MUBL_FLASH_PAGE;
    }
#endif
    unlock();
    FLASH_CR = FLASH_CR_PER | bker | (pnb << FLASH_CR_PNB_POS);
    FLASH_CR |= FLASH_CR_STRT;
    return finish();
}

bool hal_flash_program(uint32_t addr, const uint8_t* data)
{
    if (addr < MUBL_APP_BASE || (addr & 7U) != 0U) return false;
    unlock();
    FLASH_CR = FLASH_CR_PG;
    volatile uint32_t* dst = (volatile uint32_t*)(uintptr_t)addr;
    dst[0] = mubl_get32(data);
    dst[1] = mubl_get32(data + 4U);
    __asm volatile("dsb" ::: "memory");
    return finish();
}

#elif MUBL_FAMILY == MUBL_FAMILY_H7

MUBL_STATIC_ASSERT(MUBL_FLASH_GRANULE == 32, "H7 programs 256-bit flash words");
MUBL_STATIC_ASSERT(MUBL_APP_BASE % FLASH_SECTOR == 0, "the application must start on a sector");

#if MUBL_H7_FLASH_PSIZE <= 3
#define PSIZE (MUBL_H7_FLASH_PSIZE << FLASH_CR_PSIZE_POS)
#else
#define PSIZE 0U
#endif

static uint32_t bank_of(uint32_t addr)
{
    return (addr - MUBL_FLASH_BASE) / FLASH_BANK_SIZE;
}

static void idle(uint32_t b)
{
    while (FLASH_SR(b) & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)) {
    }
}

static void unlock(uint32_t b)
{
    if (FLASH_CR(b) & FLASH_CR_LOCK) {
        FLASH_KEYR(b) = FLASH_KEY1;
        FLASH_KEYR(b) = FLASH_KEY2;
    }
    idle(b);
    FLASH_CCR(b) = FLASH_CCR_ALL;
}

static bool finish(uint32_t b)
{
    idle(b);
    const bool ok = (FLASH_SR(b) & FLASH_SR_PROG_ERRORS) == 0U;
    FLASH_CCR(b) = FLASH_CCR_ALL;
    FLASH_CR(b) = FLASH_CR_LOCK;
    return ok;
}

void hal_flash_sector(uint32_t addr, uint32_t* start, uint32_t* end)
{
    *start = addr & ~(FLASH_SECTOR - 1U);
    *end = *start + FLASH_SECTOR;
}

bool hal_flash_erase(uint32_t start)
{
    if (start < MUBL_APP_BASE || start % FLASH_SECTOR != 0U) return false;
    const uint32_t b = bank_of(start), snb = ((start - MUBL_FLASH_BASE) % FLASH_BANK_SIZE) / FLASH_SECTOR;
    unlock(b);
    FLASH_CR(b) = FLASH_CR_SER | (snb << FLASH_CR_SNB_POS) | PSIZE;
    FLASH_CR(b) |= FLASH_CR_START;
    return finish(b);
}

bool hal_flash_program(uint32_t addr, const uint8_t* data)
{
    if (addr < MUBL_APP_BASE) return false;
    const uint32_t b = bank_of(addr);
    unlock(b);
    FLASH_CR(b) = FLASH_CR_PG | PSIZE;
    __asm volatile("isb\ndsb" ::: "memory");
    volatile uint32_t* dst = (volatile uint32_t*)(uintptr_t)addr;
    for (uint32_t i = 0; i < 8U; i++) dst[i] = mubl_get32(&data[4U * i]);
    __asm volatile("isb\ndsb" ::: "memory");
    return finish(b);
}

#endif
