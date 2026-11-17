/*
 * The PX4/ArduPilot serial bootloader protocol, revision 5 (the subset uploader.py, QGroundControl
 * and Mission Planner use).
 *
 *   command: <opcode> [arguments] <EOC 0x20>      reply: [data] <INSYNC 0x12> <status>
 *
 * GET_SYNC, GET_DEVICE, CHIP_ERASE, PROG_MULTI (at most 252 bytes, a multiple of 4), GET_CRC,
 * GET_OTP, GET_SN, GET_CHIP, GET_CHIP_DES, GET_VERSION, BOOT and SET_BAUD are implemented;
 * anything else gets INVALID.  As in the reference implementation, erasing and programming need
 * GET_SYNC and the BL_REV, BOARD_ID and FW_SIZE queries first (line noise is unlikely to produce
 * them), and those queries also stop the boot timeout.
 *
 * GET_CRC covers the whole application region as it will be after BOOT, so the held-back first
 * block counts even though it is not yet in flash.  BOOT validates the image and makes it
 * bootable (update_finish); it replies FAILED if the image does not pass.
 *
 * Nothing is sent until a well-formed GET_SYNC has claimed the port, so on a port shared with SRXL2
 * stray bytes never produce a reply.
 */
#include "hal/hal.h"
#include "links.h"
#include "update.h"

#if MUBL_SERIAL

#define LINK MUBL_LINK_SERIAL

#define INSYNC 0x12
#define EOC 0x20
#define OK 0x10
#define FAILED 0x11
#define INVALID 0x13

#define GET_SYNC 0x21
#define GET_DEVICE 0x22
#define CHIP_ERASE 0x23
#define PROG_MULTI 0x27
#define GET_CRC 0x29
#define GET_OTP 0x2A
#define GET_SN 0x2B
#define GET_CHIP 0x2C
#define GET_CHIP_DES 0x2E
#define GET_VERSION 0x2F
#define BOOT 0x30
#define SET_BAUD 0x33
#define CHIP_FULL_ERASE 0x40

#define BL_REV 5
#define PROG_MAX 252
#define COMMAND_TIMEOUT_MS 500U       /* a command not finished by then is dropped */
#define SESSION_TIMEOUT_MS 30000U     /* an update with no commands for this long is abandoned */
#define DEVICE_FLAGS 0x0BU            /* BL_REV, BOARD_ID, FW_SIZE asked for */

static struct {
    uint8_t op;           /* command being received, 0 if none */
    uint8_t got;          /* argument bytes received */
    uint8_t need;         /* argument bytes expected */
    uint8_t synced;
    uint8_t device;       /* GET_DEVICE arguments seen, bit n-1 for argument n */
    uint8_t owner;        /* the port is claimed */
    uint32_t last;        /* time of the last byte */
    uint32_t address;
    uint8_t arg[PROG_MAX + 1];
} s;

static void reply(const uint8_t* data, uint8_t n, uint8_t status)
{
    uint8_t out[24];
    memcpy(out, data, n);
    out[n] = INSYNC;
    out[n + 1U] = status;
    uart_send(LINK, out, (uint16_t)(n + 2U));
}

static void reply_word(uint32_t w, uint8_t status)
{
    uint8_t b[4];
    mubl_put32(b, w);
    reply(b, 4, status);
}

static void reply_string(const char* text)
{
    uint8_t b[20];
    uint8_t n = 0;
    while (text[n] != '\0' && n < 16U) {
        b[4 + n] = (uint8_t)text[n];
        n++;
    }
    mubl_put32(b, n);
    reply(b, (uint8_t)(n + 4U), OK);
}

/* Application vectors 7-10, copied out under hal_guarded: after an interrupted update the first
 * flash word can be erased (so the image check stopped there) while the next one is damaged. */
static void read_vectors(void* out)
{
    MUBL_FLASH_READ(MUBL_APP_BASE + 28U, 16U);
    memcpy(out, MUBL_FLASH_PTR(MUBL_APP_BASE + 28U), 16U);
}

static bool ready(void)
{
    return s.synced && (s.device & DEVICE_FLAGS) == DEVICE_FLAGS;
}

static uint8_t status_of(int r)
{
    return r == MUBL_OK ? OK : FAILED;
}

static void execute(uint8_t op)
{
    uint8_t uid[16];
    const uint32_t word = mubl_get32(s.arg);
    switch (op) {
    case GET_SYNC:
        s.synced = 1;
        reply(NULL, 0, OK);
        return;
    case GET_DEVICE: {
        const uint8_t a = s.arg[0];
        if (a == 5U) {           /* VEC_AREA: application vectors 7-10 */
            uint8_t v[16];
            if (!hal_guarded(read_vectors, v)) {
                reply(NULL, 0, FAILED);
                return;
            }
            reply(v, 16, OK);
        } else if (a >= 1U && a <= 6U) {
            static const uint32_t values[] = {BL_REV, MUBL_BOARD_ID, MUBL_BOARD_REV, MUBL_APP_SIZE, 0, 0};
            reply_word(values[a - 1U], OK);
        } else {
            reply(NULL, 0, INVALID);
            return;
        }
        s.device |= (uint8_t)(1U << (a - 1U));
        if (ready()) mubl_hold(LINK);
        return;
    }
    case CHIP_ERASE:
    case CHIP_FULL_ERASE:
        if (!ready()) break;
        s.address = 0;
        reply(NULL, 0, status_of(update_erase_all(LINK)));
        return;
    case PROG_MULTI: {
        const uint8_t n = s.arg[0];
        if (!ready() || n % 4U != 0U || update_owner() != LINK) break;
        const int r = update_write(LINK, s.address, &s.arg[1], n);
        if (r == MUBL_OK) s.address += n;
        reply(NULL, 0, status_of(r));
        return;
    }
    case GET_CRC: {
        uint32_t crc;
        if (!update_crc32(0, 0, MUBL_APP_SIZE, &crc)) break;
        reply_word(crc, OK);
        return;
    }
    case GET_OTP:
        reply_word(0, OK);
        return;
    case GET_SN:
        if (word > 12U) break;
        hal_unique_id(uid);
        reply(&uid[word], 4, OK);
        return;
    case GET_CHIP:
        reply_word(hal_idcode(), OK);
        return;
    case GET_CHIP_DES:
        reply_string(MUBL_SERIAL_CHIP_DES);
        return;
    case GET_VERSION:
        reply_string(MUBL_BOARD_NAME);
        return;
    case BOOT: {
        int r = MUBL_OK;
        if (update_owner() == LINK) r = update_finish(LINK);
        if (r == MUBL_OK && mubl_app()->status != MUBL_APP_OK) r = MUBL_E_IMAGE;
        reply(NULL, 0, status_of(r));
        if (r == MUBL_OK) mubl_boot(20);
        return;
    }
    case SET_BAUD:
        if (!ready() || word < 1200U) break;
        reply(NULL, 0, OK);
        uart_set_baud(LINK, word);
        mubl_session()->uart_baud = word;
        return;
    default:
        break;
    }
    reply(NULL, 0, INVALID);
}

/* Argument bytes before EOC, or -1 for an opcode we do not know. */
static int arguments(uint8_t op)
{
    switch (op) {
    case GET_SYNC: case CHIP_ERASE: case CHIP_FULL_ERASE: case GET_CRC: case GET_CHIP:
    case GET_CHIP_DES: case GET_VERSION: case BOOT:
        return 0;
    case GET_DEVICE: case PROG_MULTI:
        return 1;           /* PROG_MULTI: the length, then that many bytes */
    case GET_OTP: case GET_SN: case SET_BAUD:
        return 4;
    default:
        return -1;
    }
}

void serial_rx(uint8_t b, uint32_t now)
{
    if (s.op != 0U && mubl_since(now, s.last) > (int32_t)COMMAND_TIMEOUT_MS) s.op = 0;
    s.last = now;
    if (s.op == 0U) {
        const int n = arguments(b);
        if (n < 0) return;
        s.op = b;
        s.need = (uint8_t)n;
        s.got = 0;
        return;
    }
    if (s.got < s.need) {
        s.arg[s.got++] = b;
        if (s.op == PROG_MULTI && s.got == 1U) {
            if (b > PROG_MAX) {
                s.op = 0;
                return;
            }
            s.need = (uint8_t)(1U + b);
        }
        return;
    }
    const uint8_t op = s.op;
    s.op = 0;
    if (b != EOC) {
        if (s.owner) reply(NULL, 0, INVALID);
        return;
    }
    /* A well-formed GET_SYNC claims the port; until then nothing else is answered. */
    if (!s.owner) {
        if (op != GET_SYNC || !uart_claim(LINK)) return;
        s.owner = 1;
        mubl_detected(LINK);
    }
    execute(op);
}

void serial_poll(uint32_t now)
{
    if (update_owner() == LINK && mubl_since(now, s.last) > (int32_t)SESSION_TIMEOUT_MS) update_abort(LINK);
}

void serial_start(void)
{
    memset(&s, 0, sizeof s);
    const mubl_request_t* req = mubl_request();
    const uint32_t baud = req->link == LINK && req->uart_baud != 0U ? req->uart_baud : MUBL_SERIAL_BAUD;
    uart_listen(LINK, MUBL_SERIAL_PORT, baud);
    mubl_session()->uart_baud = baud;
}

void serial_stop(void)
{
    update_abort(LINK);
    uart_unlisten(LINK);
}

#endif
