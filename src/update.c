/*
 * Update engine: one update at a time, owned by the link that began it.
 *
 * The image is written in order.  Each sector is erased when the first byte for it arrives (an
 * already blank sector is not erased again), so the links keep running between erases and small
 * images do not pay for erasing the whole region.  The first sector is erased when the update
 * begins, which makes the old application unbootable at once.
 *
 * The first MUBL_HOLD_SIZE bytes (the initial SP and reset vector) are kept in RAM and their flash
 * left erased.  When the image is complete, sectors after it that are not blank are erased, the
 * image is validated with the held bytes substituted, and only then are the held bytes programmed,
 * last granule first, so the initial SP is the final thing written.  A reset at any point before
 * that leaves the first word erased; the bootloader then stays and the update can be repeated.
 * Once the first word is written, every other byte of the image was programmed and verified.
 */
#include "update.h"

#include "crc.h"
#include "hal/hal.h"
#include "image.h"

#define G MUBL_FLASH_GRANULE

MUBL_STATIC_ASSERT(MUBL_HOLD_SIZE % G == 0, "the held block must be whole granules");
MUBL_STATIC_ASSERT(MUBL_APP_BASE % G == 0, "the application must start on a granule");

enum { IDLE = 0, ACTIVE = 1 };

static struct {
    uint8_t state;
    uint8_t owner;
    uint32_t pos;            /* bytes written so far */
    uint32_t erased;         /* offset where the not yet erased part of the region begins */
    uint8_t first[MUBL_HOLD_SIZE];
    uint8_t pending[G];      /* bytes of the granule at pos & ~(G-1) not yet programmed */
} u;

static bool blank(uint32_t start, uint32_t end)
{
    MUBL_FLASH_READ(start, end - start);
    for (uint32_t a = start; a < end; a += 4U) {
        if (*(const uint32_t*)(const void*)MUBL_FLASH_PTR(a) != 0xFFFFFFFFU) return false;
    }
    return true;
}

typedef struct { uint32_t start, end; bool result; } blank_t;
static void blank_fn(void* arg)
{
    blank_t* b = arg;
    b->result = blank(b->start, b->end);
}

/* Erase the sector at region offset `off` unless it is already blank.  A sector that cannot even
 * be read (ECC error after an interrupted erase) is erased. */
static bool erase_sector_at(uint32_t off, uint32_t* next)
{
    blank_t b;
    hal_flash_sector(MUBL_APP_BASE + off, &b.start, &b.end);
    b.result = false;
    hal_watchdog_refresh();
    if (!hal_guarded(blank_fn, &b) || !b.result) {
        mubl_indicate(1);
        if (!hal_flash_erase(b.start) || !blank(b.start, b.end)) return false;
    }
    *next = b.end - MUBL_APP_BASE;
    return true;
}

/* Erase until the region is erased up to `end` (an offset). */
static bool erase_to(uint32_t end)
{
    while (u.erased < end) {
        if (!erase_sector_at(u.erased, &u.erased)) return false;
    }
    return true;
}

static bool program(uint32_t off, const uint8_t* data)
{
    if (!erase_to(off + G)) return false;
    if (!hal_flash_program(MUBL_APP_BASE + off, data)) return false;
    MUBL_FLASH_READ(MUBL_APP_BASE + off, G);
    return memcmp(MUBL_FLASH_PTR(MUBL_APP_BASE + off), data, G) == 0;
}

static int fail(int err)
{
    u.state = IDLE;
    return err;
}

bool update_active(void)
{
    return u.state == ACTIVE;
}

uint8_t update_owner(void)
{
    return u.state == ACTIVE ? u.owner : 0;
}

uint32_t update_position(void)
{
    return u.pos;
}

int update_begin(uint8_t owner)
{
    if (u.state == ACTIVE && u.owner != owner) return MUBL_E_BUSY;
    memset(&u, 0, sizeof u);
    memset(u.first, 0xFF, sizeof u.first);
    memset(u.pending, 0xFF, sizeof u.pending);
    u.owner = owner;
    u.state = ACTIVE;
    mubl_app_t none;
    memset(&none, 0, sizeof none);
    none.status = MUBL_APP_EMPTY;
    mubl_set_app(&none);
    /* Invalidate the old application before anything else. */
    if (!erase_to(1U)) return fail(MUBL_E_FLASH);
    return MUBL_OK;
}

int update_erase_all(uint8_t owner)
{
    int r = update_begin(owner);
    if (r != MUBL_OK) return r;
    return erase_to(MUBL_APP_SIZE) ? MUBL_OK : fail(MUBL_E_FLASH);
}

int update_write(uint8_t owner, uint32_t offset, const uint8_t* data, uint32_t n)
{
    if (u.state != ACTIVE) return MUBL_E_STATE;
    if (u.owner != owner) return MUBL_E_BUSY;
    if (offset != u.pos) return MUBL_E_SEQUENCE;
    if (n > MUBL_APP_SIZE - u.pos) return MUBL_E_RANGE;
    mubl_indicate(1);
    while (n--) {
        const uint8_t b = *data++;
        const uint32_t at = u.pos++;
        if (at < MUBL_HOLD_SIZE) {
            u.first[at] = b;
            continue;
        }
        u.pending[at % G] = b;
        if (at % G == G - 1U) {
            if (!program(at + 1U - G, u.pending)) return fail(MUBL_E_FLASH);
            memset(u.pending, 0xFF, sizeof u.pending);
        }
    }
    return MUBL_OK;
}

typedef struct { uint32_t from, to, crc; } crc_t;

/* CRC-32 of region bytes [from, to) as the image will be once finished: during an update the held
 * first block and the unprogrammed part of the last granule come from RAM. */
static void area_crc(void* arg)
{
    crc_t* c = arg;
    const bool active = u.state == ACTIVE;
    const uint32_t part = u.pos & ~(uint32_t)(G - 1U);   /* granule with unprogrammed bytes */
    const bool use_part = active && part >= MUBL_HOLD_SIZE;
    for (uint32_t off = c->from; off < c->to;) {
        const uint8_t* p;
        uint32_t end;
        if (active && off < MUBL_HOLD_SIZE) {
            p = &u.first[off];
            end = MUBL_HOLD_SIZE;
        } else if (use_part && off >= part && off < part + G) {
            p = &u.pending[off - part];
            end = part + G;
        } else {
            p = MUBL_FLASH_PTR(MUBL_APP_BASE + off);
            end = use_part && off < part ? part : c->to;
            if (end > c->to) end = c->to;
            MUBL_FLASH_READ(MUBL_APP_BASE + off, end - off);
        }
        if (end > c->to) end = c->to;
        c->crc = crc32_update(c->crc, p, end - off);
        off = end;
    }
}

bool update_crc32(uint32_t crc, uint32_t from, uint32_t to, uint32_t* out)
{
    crc_t c = {from, to, crc};
    if (to > MUBL_APP_SIZE || from > to) return false;
    if (!hal_guarded(area_crc, &c)) return false;
    *out = c.crc;
    return true;
}

int update_finish(uint8_t owner)
{
    if (u.state != ACTIVE) return MUBL_E_STATE;
    if (u.owner != owner) return MUBL_E_BUSY;
    if (u.pos < MUBL_HOLD_SIZE) return fail(MUBL_E_IMAGE);
    /* Program the last, partial granule padded with erased bytes. */
    if (u.pos % G != 0U && !program(u.pos & ~(uint32_t)(G - 1U), u.pending)) return fail(MUBL_E_FLASH);
    /* Clear whatever a previous, larger image left after this one. */
    if (!erase_to(MUBL_APP_SIZE)) return fail(MUBL_E_FLASH);
    mubl_app_t app;
    image_check(&app, u.first);
    if (app.status != MUBL_APP_OK) {
        mubl_set_app(&app);
        return fail(MUBL_E_IMAGE);
    }
    for (uint32_t off = MUBL_HOLD_SIZE; off > 0U;) {
        off -= G;
        if (!program(off, &u.first[off])) return fail(MUBL_E_FLASH);
    }
    u.state = IDLE;
    image_check(&app, NULL);
    mubl_set_app(&app);
    return app.status == MUBL_APP_OK ? MUBL_OK : MUBL_E_FLASH;
}

void update_abort(uint8_t owner)
{
    if (u.state == ACTIVE && u.owner == owner) u.state = IDLE;
}

void update_init(void)
{
    memset(&u, 0, sizeof u);
}
