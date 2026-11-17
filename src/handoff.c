/*
 * Application <-> bootloader handoff.
 *
 * Native block: MUBL_HANDOFF_SIZE bytes of retained RAM at MUBL_HANDOFF_ADDR, protected by a
 * CRC-32 and a direction-specific magic, consumed (magic cleared) as soon as it is read so a later
 * reset cannot replay it.  Layout (little endian), shared with app/mubl_app.h:
 *
 *   0  magic         MUBL_HANDOFF_TO_BL or MUBL_HANDOFF_TO_APP
 *   4  crc           zlib CRC-32 of the `length` bytes from offset 8
 *   8  length        20 + path_len
 *   10 version       1
 *   11 flags         to the bootloader: bit 0 stay in the bootloader
 *   12 link          to the bootloader: link to use (0: detect); to the app: link used (0: none)
 *   13 boot_attempts to the app: boots the application has not confirmed (it clears the magic)
 *   14 can_node_id
 *   15 can_server_node_id   with a path: the DroneCAN file server holding the new image
 *   16 can_bitrate
 *   20 uart_baud
 *   24 srxl2_device_id
 *   25 reserved[2]
 *   27 path_len      0..200
 *   28 path
 *
 * Compatibility: AP_Periph's app_bootloader_comms (MUBL_COMPAT_AP_PERIPH) and the PX4/thiemar
 * structure in bxCAN filter registers (MUBL_COMPAT_PX4_CAN, see hal_px4_shared_*).
 */
#include "handoff.h"

#include "crc.h"
#include "hal/hal.h"

#define FIXED 20U
#define FLAG_STAY 1U

MUBL_STATIC_ASSERT(MUBL_HANDOFF_SIZE >= 8 + FIXED + 200, "handoff block too small for a path");

static uint32_t zcrc(const uint8_t* p, uint32_t n)
{
    return ~crc32_update(~0U, p, n);
}

#if MUBL_COMPAT_AP_PERIPH
/* ArduPilot Tools/AP_Bootloader/app_comms.h */
#define AP_COMMS_MAGIC 0xC544AD9AU
static void read_ap_periph(mubl_request_t* req)
{
    uint8_t* c = MUBL_RAM_PTR(MUBL_AP_COMMS_ADDR);
    if (mubl_get32(c) != AP_COMMS_MAGIC) return;
    mubl_put32(c, 0);
    req->valid = 1;
    req->stay = 1;
    req->link = MUBL_LINK_DRONECAN;
    req->can_server_node_id = c[20];
    req->can_node_id = c[21];
    uint8_t n = 0;
    while (n < sizeof req->path && c[22 + n] != 0U) {
        req->path[n] = c[22 + n];
        n++;
    }
    req->path_len = n;
}
#endif

#if MUBL_COMPAT_PX4_CAN
#define PX4_APP_SIGNATURE 0xB0A04150U
#define PX4_BL_SIGNATURE 0xB0A0424CU

/* CRC-64-WE of signature, bus_speed, node_id; w = {crc_hi, crc_lo, signature, bus_speed, node_id}. */
static uint64_t px4_crc(const uint32_t* w)
{
    uint8_t b[12];
    mubl_put32(b, w[2]);
    mubl_put32(b + 4, w[3]);
    mubl_put32(b + 8, w[4]);
    return ~crc64_update(~0ULL, b, sizeof b);
}

static void read_px4(mubl_request_t* req)
{
    uint32_t w[5];
    hal_px4_shared_read(w);
    const uint64_t crc = ((uint64_t)w[0] << 32) | w[1];
    if (w[2] != PX4_APP_SIGNATURE || crc != px4_crc(w)) return;
    hal_px4_shared_write(NULL);
    req->valid = 1;
    req->stay = 1;
    req->link = MUBL_LINK_DRONECAN;
    req->can_bitrate = w[3];
    req->can_node_id = (uint8_t)w[4];
}
#endif

void handoff_read(mubl_request_t* req)
{
    uint8_t* h = MUBL_RAM_PTR(MUBL_HANDOFF_ADDR);
    const uint32_t magic = mubl_get32(h);
    const uint16_t len = mubl_get16(h + 8);
    memset(req, 0, sizeof *req);
    if ((magic == MUBL_HANDOFF_TO_BL || magic == MUBL_HANDOFF_TO_APP) && len >= FIXED &&
        len <= MUBL_HANDOFF_SIZE - 8U && h[10] == 1U && zcrc(h + 8, len) == mubl_get32(h + 4)) {
        if (magic == MUBL_HANDOFF_TO_BL) {
            uint8_t n = h[27];
            if (n > sizeof req->path) n = sizeof req->path;
            if (n > len - FIXED) n = (uint8_t)(len - FIXED);
            req->valid = 1;
            req->stay = h[11] & FLAG_STAY;
            req->link = h[12] < MUBL_LINK_COUNT ? h[12] : 0;
            req->can_node_id = h[14] & 0x7FU;
            req->can_server_node_id = h[15] & 0x7FU;
            req->can_bitrate = mubl_get32(h + 16);
            req->uart_baud = mubl_get32(h + 20);
            req->srxl2_device_id = h[24];
            req->path_len = n;
            memcpy(req->path, h + 28, n);
        } else {
            req->boot_attempts = h[13];    /* our own block: the application never confirmed */
        }
    }
    mubl_put32(h, 0);
#if MUBL_COMPAT_AP_PERIPH
    if (!req->valid) read_ap_periph(req);
#endif
#if MUBL_COMPAT_PX4_CAN
    if (!req->valid) read_px4(req);
#endif
}

void handoff_write(const mubl_session_t* s, uint8_t boot_attempts)
{
    uint8_t* h = MUBL_RAM_PTR(MUBL_HANDOFF_ADDR);
    memset(h, 0, 8U + FIXED);
    mubl_put32(h, MUBL_HANDOFF_TO_APP);
    h[8] = FIXED;
    h[10] = 1;
    h[12] = s->link;
    h[13] = boot_attempts;
    h[14] = s->can_node_id;
    mubl_put32(h + 16, s->can_bitrate);
    mubl_put32(h + 20, s->uart_baud);
    h[24] = s->srxl2_device_id;
    mubl_put32(h + 4, zcrc(h + 8, FIXED));
#if MUBL_COMPAT_PX4_CAN
    if (s->can_node_id != 0U) {
        uint32_t w[5] = {0, 0, PX4_BL_SIGNATURE, s->can_bitrate, s->can_node_id};
        const uint64_t crc = px4_crc(w);
        w[0] = (uint32_t)(crc >> 32);
        w[1] = (uint32_t)crc;
        hal_px4_shared_write(w);
    }
#endif
}
