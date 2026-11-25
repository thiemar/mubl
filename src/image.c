/*
 * Application image validation.
 *
 * An image is bootable when its initial SP is in RAM this board has (MUBL_RAM_BASE/SIZE or a bank
 * listed in MUBL_STACK_RAM), its reset vector is Thumb code in the application region and, if it
 * carries a descriptor, the descriptor's size is inside the region, its CRCs (and for the ArduPilot
 * format, its board ID) match, and the reset vector is inside the image it describes.  An image without a descriptor is accepted only when
 * MUBL_IMG_RAW is enabled.
 *
 * Every bound is checked against the application region before anything is read, so a corrupt
 * descriptor cannot send the CRC loop outside it.  On parts with flash ECC the whole check runs
 * under hal_guarded: an uncorrectable error makes the image invalid instead of faulting the
 * bootloader into a reset loop.
 */
#include "image.h"

#include "crc.h"
#include "hal/hal.h"

#ifndef MUBL_STACK_RAM
#define MUBL_STACK_RAM
#endif
#ifndef MUBL_BOARD_ID_ALT
#define MUBL_BOARD_ID_ALT MUBL_BOARD_ID
#endif

#define AP_SIG_LO 0xF1E4A240U       /* 40 a2 e4 f1 64 68 91 06 */
#define AP_SIG_HI 0x06916864U
#define LEGACY_SIG_LO 0x65445041U   /* "APDesc00" */
#define LEGACY_SIG_HI 0x30306373U
#define AP_DESC_SIZE 36U
#define LEGACY_DESC_SIZE 32U

typedef struct {
    mubl_app_t* app;
    const uint8_t* first;
} check_t;

/* A full-descending stack starts at most at the end of a RAM bank, and above its start. */
static bool sp_valid(uint32_t sp)
{
    static const struct { uint32_t start, end; } ram[] = {
        {MUBL_RAM_BASE, MUBL_RAM_BASE + MUBL_RAM_SIZE}, MUBL_STACK_RAM
    };
    if ((sp & 3U) != 0U) return false;
    for (uint32_t i = 0; i < sizeof ram / sizeof ram[0]; i++) {
        if (sp > ram[i].start && sp <= ram[i].end) return true;
    }
    return false;
}

/* With a descriptor the image's extent is known: the reset vector must be inside it. */
static void entry_in_image(mubl_app_t* app, uint32_t entry)
{
    if (app->status == MUBL_APP_OK && (entry & ~1U) - MUBL_APP_BASE >= app->size) app->status = MUBL_APP_BAD_VECTORS;
}

static uint32_t word(uint32_t off)
{
    MUBL_FLASH_READ(MUBL_APP_BASE + off, 4U);
    return *(const uint32_t*)(const void*)MUBL_FLASH_PTR(MUBL_APP_BASE + off);
}

#if MUBL_IMAGE_FORMATS & MUBL_IMG_AP
/* CRC-32 of image bytes [from, to): the first MUBL_HOLD_SIZE bytes come from `first`. */
static uint32_t image_crc32(const uint8_t* first, uint32_t crc, uint32_t from, uint32_t to)
{
    if (from < MUBL_HOLD_SIZE && from < to) {
        const uint32_t end = to < MUBL_HOLD_SIZE ? to : MUBL_HOLD_SIZE;
        crc = crc32_update(crc, first + from, end - from);
        from = end;
    }
    if (from < to) {
        MUBL_FLASH_READ(MUBL_APP_BASE + from, to - from);
        crc = crc32_update(crc, MUBL_FLASH_PTR(MUBL_APP_BASE + from), to - from);
    }
    return crc;
}

static void check_ap(mubl_app_t* app, const uint8_t* first, uint32_t off)
{
    const uint32_t size = word(off + 16U);
    app->format = MUBL_FMT_AP;
    if (size > MUBL_APP_SIZE || size < off + AP_DESC_SIZE) {
        app->status = MUBL_APP_BAD_SIZE;
        return;
    }
    const uint32_t meta = word(off + 24U);          /* major, minor, board_id */
    app->board_id = (uint16_t)(meta >> 16);
#if MUBL_CHECK_BOARD_ID
    if (app->board_id != (uint16_t)MUBL_BOARD_ID && app->board_id != (uint16_t)MUBL_BOARD_ID_ALT) {
        app->status = MUBL_APP_BAD_BOARD;
        return;
    }
#endif
    /* crc1 covers the image up to and including the signature; crc2 starts at version_major. */
    const uint32_t crc1 = image_crc32(first, 0, 0, off + 8U);
    const uint32_t crc2 = image_crc32(first, 0, off + 24U, size);
    app->crc = ((uint64_t)crc2 << 32) | crc1;
    if (crc1 != word(off + 8U) || crc2 != word(off + 12U)) {
        app->status = MUBL_APP_BAD_CRC;
        return;
    }
    app->size = size;
    app->vcs_commit = word(off + 20U);
    app->major = (uint8_t)meta;
    app->minor = (uint8_t)(meta >> 8);
    app->status = MUBL_APP_OK;
}
#endif

#if MUBL_IMAGE_FORMATS & MUBL_IMG_LEGACY
static uint64_t image_crc64(const uint8_t* first, uint64_t crc, uint32_t from, uint32_t to)
{
    if (from < MUBL_HOLD_SIZE && from < to) {
        const uint32_t end = to < MUBL_HOLD_SIZE ? to : MUBL_HOLD_SIZE;
        crc = crc64_update(crc, first + from, end - from);
        from = end;
    }
    if (from < to) {
        MUBL_FLASH_READ(MUBL_APP_BASE + from, to - from);
        crc = crc64_update(crc, MUBL_FLASH_PTR(MUBL_APP_BASE + from), to - from);
    }
    return crc;
}

static void check_legacy(mubl_app_t* app, const uint8_t* first, uint32_t off)
{
    static const uint8_t zero[8];
    const uint32_t size = word(off + 16U);
    app->format = MUBL_FMT_LEGACY;
    if (size > MUBL_APP_SIZE || size < off + LEGACY_DESC_SIZE) {
        app->status = MUBL_APP_BAD_SIZE;
        return;
    }
    /* CRC-64-WE of the whole image with the image_crc field read as zero. */
    uint64_t crc = image_crc64(first, ~0ULL, 0, off + 8U);
    crc = crc64_update(crc, zero, 8U);
    crc = ~image_crc64(first, crc, off + 16U, size);
    app->crc = crc;
    if (crc != ((uint64_t)word(off + 12U) << 32 | word(off + 8U))) {
        app->status = MUBL_APP_BAD_CRC;
        return;
    }
    const uint32_t ver = word(off + 24U);
    app->size = size;
    app->vcs_commit = word(off + 20U);
    app->major = (uint8_t)ver;
    app->minor = (uint8_t)(ver >> 8);
    app->status = MUBL_APP_OK;
}
#endif

static void check(void* arg)
{
    check_t* c = arg;
    mubl_app_t* app = c->app;
    const uint8_t* first = c->first;
    if (first == NULL) {
        MUBL_FLASH_READ(MUBL_APP_BASE, MUBL_HOLD_SIZE);
        first = MUBL_FLASH_PTR(MUBL_APP_BASE);
    }
    const uint32_t sp = mubl_get32(first), entry = mubl_get32(first + 4);
    app->status = MUBL_APP_EMPTY;
    if (sp == 0xFFFFFFFFU) return;
    app->status = MUBL_APP_BAD_VECTORS;
    if (!sp_valid(sp) || (entry & 1U) == 0U || entry < MUBL_APP_BASE ||
        entry - MUBL_APP_BASE >= MUBL_APP_SIZE) {
        return;
    }
    /* The descriptor follows the vector table, word aligned.  The first match is the image's own;
     * flash after the image is erased by every update, so a stale descriptor cannot be found. */
    const uint32_t limit = MUBL_DESC_SCAN_LIMIT < MUBL_APP_SIZE ? MUBL_DESC_SCAN_LIMIT : MUBL_APP_SIZE;
    for (uint32_t off = (MUBL_HOLD_SIZE + 3U) & ~3U; off + AP_DESC_SIZE <= limit; off += 4U) {
        const uint32_t lo = word(off);
#if MUBL_IMAGE_FORMATS & MUBL_IMG_AP
        if (lo == AP_SIG_LO && word(off + 4U) == AP_SIG_HI) {
            check_ap(app, first, off);
            entry_in_image(app, entry);
            return;
        }
#endif
#if MUBL_IMAGE_FORMATS & MUBL_IMG_LEGACY
        if (lo == LEGACY_SIG_LO && word(off + 4U) == LEGACY_SIG_HI) {
            check_legacy(app, first, off);
            entry_in_image(app, entry);
            return;
        }
#endif
    }
#if MUBL_IMAGE_FORMATS & MUBL_IMG_RAW
    app->format = MUBL_FMT_RAW;
    app->status = MUBL_APP_OK;
#else
    app->status = MUBL_APP_NO_DESCRIPTOR;
#endif
}

void image_check(mubl_app_t* app, const uint8_t* first)
{
    check_t c = {app, first};
    memset(app, 0, sizeof *app);
    if (!hal_guarded(check, &c)) {
        memset(app, 0, sizeof *app);
        app->status = MUBL_APP_FLASH_ERROR;
    }
}
