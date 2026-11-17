/*
 * Spektrum SRXL2 (specification Rev K), as a slave device, plus a µBL packet for updates.
 *
 * Packets: <0xA6> <type> <length> <payload> <CRC-16 XMODEM, big endian>, 5 to 80 bytes, on a
 * single-wire half-duplex line at 115200 baud (400000 once the bus master says so).
 *
 * Handshake (7.2.1): listen for 50 ms after start; if nothing was heard and the unit ID nibble is
 * 0, announce with a Handshake to device 0 every 50 ms for 200 ms.  Answer a Handshake addressed
 * to us; on a Handshake to 0xFF switch to the baud rate it gives.  If the master is silent for
 * 50 ms, return to 115200.
 *
 * Update packet (type MUBL_SRXL2_DFU_TYPE; the public specification defines no update packet):
 *
 *   master -> device: <dest> <src> <op> <seq> <data...>
 *   device -> master: <master> <device> <op | 0x80> <seq> <status> <data...>
 *
 *   op 1 INFO                     -> app status, board ID, region size, versions, unique ID
 *   op 2 BEGIN  [size:4]          erase the first sector (the old application is then gone)
 *   op 3 WRITE  <offset:4> <data: 1..64>  -> next offset:4
 *   op 4 FINISH <size:4> <crc:4>  zlib CRC-32 of the image; validate and make bootable -> app status
 *   op 5 BOOT
 *
 * Status is a MUBL_OK/MUBL_E_* code.  The device answers only packets addressed to it.  A packet
 * repeating the previous one's src, op and seq (a retry after a lost reply) gets the previous
 * reply again without being executed twice.  Any packet to us holds the bootloader.
 */
#include "crc.h"
#include "hal/hal.h"
#include "links.h"
#include "update.h"

#if MUBL_SRXL2

#define LINK MUBL_LINK_SRXL2

#define SRXL_ID 0xA6
#define TYPE_HANDSHAKE 0x21
#define BAUD_DEFAULT 115200U
#define BAUD_FAST 400000U
#define GAP_MS 2U
#define LISTEN_MS 50U
#define ANNOUNCE_MS 200U
#define SILENCE_MS 50U
#define SESSION_SILENCE_MS 2000U
#define SESSION_TIMEOUT_MS 30000U
#define MAX_PACKET 80U

#define OP_INFO 1
#define OP_BEGIN 2
#define OP_WRITE 3
#define OP_FINISH 4
#define OP_BOOT 5
#define PROTOCOL_VERSION 1

enum { LISTEN = 0, ANNOUNCE = 1, RUN = 2 };

static struct {
    uint8_t state;
    uint8_t id;
    uint8_t owner;
    uint8_t heard;            /* a packet arrived since the last silence */
    uint8_t len;
    uint8_t last_valid, last_src, last_op, last_seq, last_len;
    uint32_t started, next_announce, last_rx, last_byte, last_dfu;
    uint32_t uid;
    uint8_t buf[MAX_PACKET];
    uint8_t last[48];         /* the previous update reply, for retries */
} s;

static void send(uint8_t* p, uint8_t n)
{
    p[0] = SRXL_ID;
    p[2] = n;
    const uint16_t crc = crc16_xmodem(0, p, n - 2U);
    p[n - 2U] = (uint8_t)(crc >> 8);
    p[n - 1U] = (uint8_t)crc;
    hal_delay_us(MUBL_SRXL2_REPLY_DELAY_US);
    uart_send(LINK, p, n);
}

static void handshake(uint8_t dest)
{
    uint8_t p[14];
    p[1] = TYPE_HANDSHAKE;
    p[3] = s.id;
    p[4] = dest;
    p[5] = 1;                         /* telemetry priority: a bootloader has none to send */
    p[6] = MUBL_SRXL2_400K ? 1 : 0;
    p[7] = 0;                         /* info: no RF telemetry, not a forward-programming target */
    mubl_put32(&p[8], s.uid);
    send(p, sizeof p);
}

static uint8_t info(uint8_t* r)
{
    const mubl_app_t* app = mubl_app();
    uint8_t uid[16];
    hal_unique_id(uid);
    r[1] = PROTOCOL_VERSION;
    r[2] = app->status;
    r[3] = app->format;
    mubl_put32(&r[4], MUBL_BOARD_ID);
    mubl_put32(&r[8], MUBL_APP_SIZE);
    r[12] = app->major;
    r[13] = app->minor;
    mubl_put32(&r[14], app->vcs_commit);
    mubl_put32(&r[18], app->size);
    memcpy(&r[22], uid, 12);
    r[34] = MUBL_VERSION_MAJOR;
    r[35] = MUBL_VERSION_MINOR;
    return 36;
}

static void dfu(uint8_t src, uint8_t op, uint8_t seq, const uint8_t* d, uint8_t n, uint32_t now)
{
    uint8_t p[MAX_PACKET];
    uint8_t* r = &p[7];
    uint8_t rn = 1;
    int status = MUBL_OK;

    s.last_dfu = now;
    mubl_hold(LINK);
    if (op != OP_INFO && s.last_valid && src == s.last_src && op == s.last_op && seq == s.last_seq) {
        memcpy(p, s.last, s.last_len);
        send(p, s.last_len);
        return;
    }
    switch (op) {
    case OP_INFO:
        rn = info(r);
        break;
    case OP_BEGIN:
        status = n >= 4U && mubl_get32(d) > MUBL_APP_SIZE ? MUBL_E_RANGE : update_begin(LINK);
        break;
    case OP_WRITE:
        status = n < 5U ? MUBL_E_ARGUMENT : update_write(LINK, mubl_get32(d), d + 4, n - 4U);
        mubl_put32(&r[1], update_position());
        rn = 5;
        break;
    case OP_FINISH: {
        uint32_t crc;
        if (n < 8U) {
            status = MUBL_E_ARGUMENT;
        } else if (update_owner() != LINK) {
            status = update_active() ? MUBL_E_BUSY : MUBL_E_STATE;
        } else if (mubl_get32(d) != update_position()) {
            status = MUBL_E_SEQUENCE;
        } else if (!update_crc32(~0U, 0, update_position(), &crc) || ~crc != mubl_get32(d + 4)) {
            update_abort(LINK);
            status = MUBL_E_IMAGE;
        } else {
            status = update_finish(LINK);
        }
        r[1] = mubl_app()->status;
        rn = 2;
        break;
    }
    case OP_BOOT:
        status = update_active() || mubl_app()->status != MUBL_APP_OK ? MUBL_E_IMAGE : MUBL_OK;
        if (status == MUBL_OK) mubl_boot(20);
        break;
    default:
        status = MUBL_E_ARGUMENT;
        break;
    }
    r[0] = (uint8_t)status;
    p[1] = MUBL_SRXL2_DFU_TYPE;
    p[3] = src;
    p[4] = s.id;
    p[5] = (uint8_t)(op | 0x80U);
    p[6] = seq;
    const uint8_t len = (uint8_t)(7U + rn + 2U);
    send(p, len);
    /* INFO starts every session and is harmless to repeat: it resets retry detection, so a new
     * session reusing old sequence numbers is never mistaken for a retry. */
    s.last_valid = op != OP_INFO;
    s.last_src = src;
    s.last_op = op;
    s.last_seq = seq;
    s.last_len = len;
    memcpy(s.last, p, len < sizeof s.last ? len : sizeof s.last);
    if (len > sizeof s.last) s.last_valid = 0;
}

static void packet(uint32_t now)
{
    const uint8_t* b = s.buf;
    const uint8_t n = s.len;
    if (!s.owner) {
        if (!uart_claim(LINK)) return;
        s.owner = 1;
        mubl_detected(LINK);
    }
    s.heard = 1;
    s.last_rx = now;
    if (s.state != RUN) s.state = RUN;      /* the bus is already running: skip announcing */
    if (b[1] == TYPE_HANDSHAKE && n == 14U) {
        if (b[4] == s.id) handshake(b[3]);
        if (b[4] == 0xFFU) uart_set_baud(LINK, (b[6] & 1U) && MUBL_SRXL2_400K ? BAUD_FAST : BAUD_DEFAULT);
        mubl_session()->uart_baud = uart_baud(LINK);
    } else if (b[1] == MUBL_SRXL2_DFU_TYPE && n >= 9U && b[3] == s.id) {
        dfu(b[4], b[5], b[6], &b[7], (uint8_t)(n - 9U), now);
        s.last_rx = mubl_now();               /* time spent erasing is not the master's silence */
    }
}

void srxl2_rx(uint8_t c, uint32_t now)
{
    if (s.len != 0U && mubl_since(now, s.last_byte) > (int32_t)GAP_MS) s.len = 0;
    s.last_byte = now;
    if (s.len == 0U && c != SRXL_ID) return;
    if (s.len == 2U && (c < 5U || c > MAX_PACKET)) {
        s.len = 0;
        return;
    }
    s.buf[s.len++] = c;
    if (s.len < 3U || s.len < s.buf[2]) return;
    const uint8_t n = s.len;
    s.len = 0;
    if (crc16_xmodem(0, s.buf, n) != 0U) return;    /* the CRC over everything, its own included */
    s.len = n;
    packet(now);
    s.len = 0;
}

void srxl2_poll(uint32_t now)
{
    switch (s.state) {
    case LISTEN:
        if (mubl_since(now, s.started) < (int32_t)LISTEN_MS) break;
        s.state = (s.id & 0x0FU) == 0U ? ANNOUNCE : RUN;
        s.next_announce = now;
        break;
    case ANNOUNCE:
        if (mubl_since(now, s.started) >= (int32_t)(LISTEN_MS + ANNOUNCE_MS)) {
            s.state = RUN;
        } else if ((int32_t)(now - s.next_announce) >= 0) {
            s.next_announce = now + 50U;
            if (uart_may_send(LINK)) handshake(0);
        }
        break;
    default:
        /* The master went quiet: it may have browned out, so return to 115200 and handshake again
         * (7.2.1 step 6).  During an update the master is our updater, which may pause between
         * requests, so it gets longer. */
        if (s.heard && mubl_since(now, s.last_rx) > (int32_t)(update_owner() == LINK ? SESSION_SILENCE_MS : SILENCE_MS)) {
            s.heard = 0;
            uart_set_baud(LINK, BAUD_DEFAULT);
            mubl_session()->uart_baud = BAUD_DEFAULT;
            if (update_owner() != LINK) {
                s.state = LISTEN;
                s.started = now;
            }
        }
        break;
    }
    if (update_owner() == LINK && mubl_since(now, s.last_dfu) > (int32_t)SESSION_TIMEOUT_MS) update_abort(LINK);
}

void srxl2_start(void)
{
    uint8_t uid[16];
    memset(&s, 0, sizeof s);
    const mubl_request_t* req = mubl_request();
    s.id = req->srxl2_device_id != 0U ? req->srxl2_device_id : MUBL_SRXL2_DEVICE_ID;
    hal_unique_id(uid);
    s.uid = mubl_get32(uid) ^ mubl_get32(uid + 4) ^ mubl_get32(uid + 8);
    s.started = mubl_now();
    uart_listen(LINK, MUBL_SRXL2_PORT, BAUD_DEFAULT);
    mubl_session()->srxl2_device_id = s.id;
}

void srxl2_stop(void)
{
    update_abort(LINK);
    uart_unlisten(LINK);
}

#endif
