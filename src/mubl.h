/*
 * µBL core: types and the services the boot manager offers the link protocols.
 */
#ifndef MUBL_H
#define MUBL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "board.h"
#include "mubl_config.h"

/* ---- Results ------------------------------------------------------------------------------ */

enum {
    MUBL_OK = 0,
    MUBL_E_BUSY = 1,        /* another link owns the update */
    MUBL_E_SEQUENCE = 2,    /* data is not at the next offset expected */
    MUBL_E_RANGE = 3,       /* beyond the application region */
    MUBL_E_FLASH = 4,       /* erase, program or verify failed */
    MUBL_E_IMAGE = 5,       /* the image failed validation; it stays unbootable */
    MUBL_E_STATE = 6,       /* no update in progress */
    MUBL_E_ARGUMENT = 7,
};

/* Application check results, reported over every link (DroneCAN vendor status, SRXL2 INFO). */
enum {
    MUBL_APP_OK = 0,
    MUBL_APP_EMPTY = 1,           /* first word erased: none, or an update did not finish */
    MUBL_APP_BAD_VECTORS = 2,     /* initial SP or reset vector implausible */
    MUBL_APP_NO_DESCRIPTOR = 3,   /* no descriptor, and raw images are not accepted */
    MUBL_APP_BAD_SIZE = 4,        /* descriptor size outside the application region */
    MUBL_APP_BAD_CRC = 5,
    MUBL_APP_BAD_BOARD = 6,       /* descriptor board ID is not this board's */
    MUBL_APP_FLASH_ERROR = 7,     /* uncorrectable flash error while reading the image */
};

/* Image formats found (mubl_app_t.format). */
enum { MUBL_FMT_NONE = 0, MUBL_FMT_RAW = 1, MUBL_FMT_LEGACY = 2, MUBL_FMT_AP = 3 };

typedef struct {
    uint8_t status;          /* MUBL_APP_* */
    uint8_t format;          /* MUBL_FMT_* */
    uint8_t major, minor;
    uint16_t board_id;
    uint32_t vcs_commit;
    uint32_t size;
    uint64_t crc;            /* LEGACY: CRC-64; AP: crc2 << 32 | crc1 */
} mubl_app_t;

/* Why the bootloader stays instead of booting (mubl_stay_reason()). */
enum {
    MUBL_STAY_NO = 0,
    MUBL_STAY_NO_APP = 1,         /* no valid application */
    MUBL_STAY_REQUESTED = 2,      /* the application asked (handoff) */
    MUBL_STAY_HOLD_PIN = 3,
    MUBL_STAY_HOST = 4,           /* a host started talking to a link */
    MUBL_STAY_BOOT_LOOP = 5,      /* the application did not confirm MUBL_MAX_BOOT_ATTEMPTS boots */
    MUBL_STAY_USB = 6,            /* a USB host is connected (MUBL_USB_STAY) */
    MUBL_STAY_ALLOC = 7,          /* waiting for DroneCAN node ID allocation (MUBL_DC_WAIT_ALLOC) */
};

/* What the application passed in (handoff.c), consumed by the links at start. */
typedef struct {
    uint8_t valid;               /* a request was found */
    uint8_t stay;                /* stay in the bootloader */
    uint8_t link;                /* MUBL_LINK_*: use only this link (0: detect) */
    uint8_t can_node_id;
    uint8_t can_server_node_id;  /* with path: start reading the image at once */
    uint8_t srxl2_device_id;
    uint8_t path_len;
    uint8_t boot_attempts;       /* unconfirmed boots so far */
    uint32_t can_bitrate;
    uint32_t uart_baud;
    uint8_t path[200];
} mubl_request_t;

/* What the bootloader passes to the application. */
typedef struct {
    uint8_t link;                /* link last used, 0 if none was detected */
    uint8_t can_node_id;
    uint8_t srxl2_device_id;
    uint32_t can_bitrate;
    uint32_t uart_baud;
} mubl_session_t;

/* ---- Services for link protocols ---------------------------------------------------------- */

uint32_t mubl_now(void);                   /* milliseconds since reset */
void mubl_detected(uint8_t link);          /* a peer is on this link */
void mubl_hold(uint8_t link);              /* a host is using this link: do not boot on timeout */
void mubl_usb_host(bool present);          /* a USB host appeared or went away (MUBL_USB_STAY) */
void mubl_wait_alloc(bool waiting);        /* hold boot until DroneCAN has a node ID (MUBL_DC_WAIT_ALLOC) */
void mubl_boot(uint32_t delay_ms);         /* boot the application if valid, after delay_ms */
void mubl_restart(uint32_t delay_ms);      /* reset the MCU after delay_ms */
const mubl_app_t* mubl_app(void);
void mubl_set_app(const mubl_app_t* app);  /* the update engine changed the application */
uint8_t mubl_stay_reason(void);
const mubl_request_t* mubl_request(void);  /* the application's handoff request */
mubl_session_t* mubl_session(void);        /* filled in by links, passed to the application */
uint32_t mubl_random(void);
void mubl_indicate(uint8_t activity);      /* LED; activity 1 toggles while updating */

/* Run the bootloader: mubl_init once after reset, then mubl_step forever (main() does this). */
void mubl_init(void);
void mubl_step(void);

/* Milliseconds from `t` to `now`, signed: a timestamp taken after `now` was read (a link that
 * blocked, a reply that took time) gives a small negative value, not four billion. */
static inline int32_t mubl_since(uint32_t now, uint32_t t)
{
    return (int32_t)(now - t);
}

/* ---- Utilities (no C library on targets: util.c) ------------------------------------------ */

#ifdef MUBL_SIM
#include <string.h>
#else
void* memcpy(void* dst, const void* src, size_t n);
void* memset(void* dst, int c, size_t n);
int memcmp(const void* a, const void* b, size_t n);
#endif

static inline uint16_t mubl_get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t mubl_get32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void mubl_put32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* Application flash and retained RAM as seen by the core.  The host simulation maps them onto
 * arrays; on hardware they are the addresses themselves. */
#ifdef MUBL_SIM
const uint8_t* sim_flash_ptr(uint32_t addr);
uint8_t* sim_ram_ptr(uint32_t addr);
void sim_flash_read_hook(uint32_t addr, uint32_t n);
#define MUBL_FLASH_PTR(a) sim_flash_ptr(a)
#define MUBL_RAM_PTR(a) sim_ram_ptr(a)
#define MUBL_FLASH_READ(a, n) sim_flash_read_hook((a), (n))
#else
#define MUBL_FLASH_PTR(a) ((const uint8_t*)(uintptr_t)(a))
#define MUBL_RAM_PTR(a) ((uint8_t*)(uintptr_t)(a))
#define MUBL_FLASH_READ(a, n) ((void)0)
#endif

#define MUBL_STATIC_ASSERT(c, m) _Static_assert(c, m)

#endif /* MUBL_H */
