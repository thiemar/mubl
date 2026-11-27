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
 *
 * The protocol runs on a UART (MUBL_LINK_SERIAL) and over USB CDC-ACM (MUBL_LINK_USB), one
 * instance each; only one of them can own an update.  SET_BAUD is acknowledged and ignored on USB.
 */
#include "hal/hal.h"
#include "links.h"
#include "update.h"

#if MUBL_SERIAL || MUBL_USB

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

/* One protocol instance per link: the UART (MUBL_LINK_SERIAL) and USB (MUBL_LINK_USB). */
typedef struct {
    uint8_t link;
    uint8_t op;           /* command being received, 0 if none */
    uint8_t got;          /* argument bytes received */
    uint8_t need;         /* argument bytes expected */
    uint8_t synced;
    uint8_t device;       /* GET_DEVICE arguments seen, bit n-1 for argument n */
    uint8_t owner;        /* the port is claimed */
    uint32_t last;        /* time of the last byte */
    uint32_t address;
    uint8_t arg[PROG_MAX + 1];
} serial_t;

#if MUBL_SERIAL && MUBL_USB
static serial_t inst[2];
#define UART_INST (&inst[0])
#define USB_INST (&inst[1])
#else
static serial_t inst[1];
#define UART_INST (&inst[0])
#define USB_INST (&inst[0])
#endif

static void reply(serial_t* s, const uint8_t* data, uint8_t n, uint8_t status)
{
    uint8_t out[24];
    if (n != 0U) memcpy(out, data, n);   /* status-only replies pass no data (NULL) */
    out[n] = INSYNC;
    out[n + 1U] = status;
#if MUBL_USB
    if (s->link == MUBL_LINK_USB) {
        (void)hal_usb_write(out, (uint16_t)(n + 2U));
        return;
    }
#endif
#if MUBL_SERIAL
    uart_send(s->link, out, (uint16_t)(n + 2U));
#endif
}

static void reply_word(serial_t* s, uint32_t w, uint8_t status)
{
    uint8_t b[4];
    mubl_put32(b, w);
    reply(s, b, 4, status);
}

static void reply_string(serial_t* s, const char* text)
{
    uint8_t b[20];
    uint8_t n = 0;
    while (text[n] != '\0' && n < 16U) {
        b[4 + n] = (uint8_t)text[n];
        n++;
    }
    mubl_put32(b, n);
    reply(s, b, (uint8_t)(n + 4U), OK);
}

/* Application vectors 7-10, copied out under hal_guarded: after an interrupted update the first
 * flash word can be erased (so the image check stopped there) while the next one is damaged. */
static void read_vectors(void* out)
{
    MUBL_FLASH_READ(MUBL_APP_BASE + 28U, 16U);
    memcpy(out, MUBL_FLASH_PTR(MUBL_APP_BASE + 28U), 16U);
}

static bool ready(const serial_t* s)
{
    return s->synced && (s->device & DEVICE_FLAGS) == DEVICE_FLAGS;
}

static uint8_t status_of(int r)
{
    return r == MUBL_OK ? OK : FAILED;
}

static void execute(serial_t* s, uint8_t op)
{
    const uint8_t link = s->link;
    uint8_t uid[16];
    const uint32_t word = mubl_get32(s->arg);
    switch (op) {
    case GET_SYNC:
        s->synced = 1;
        reply(s, NULL, 0, OK);
        return;
    case GET_DEVICE: {
        const uint8_t a = s->arg[0];
        if (a == 5U) {           /* VEC_AREA: application vectors 7-10 */
            uint8_t v[16];
            if (!hal_guarded(read_vectors, v)) {
                reply(s, NULL, 0, FAILED);
                return;
            }
            reply(s, v, 16, OK);
        } else if (a >= 1U && a <= 6U) {
            static const uint32_t values[] = {BL_REV, MUBL_BOARD_ID, MUBL_BOARD_REV, MUBL_APP_SIZE, 0, 0};
            reply_word(s, values[a - 1U], OK);
        } else {
            reply(s, NULL, 0, INVALID);
            return;
        }
        s->device |= (uint8_t)(1U << (a - 1U));
        if (ready(s)) mubl_hold(link);
        return;
    }
    case CHIP_ERASE:
    case CHIP_FULL_ERASE:
        if (!ready(s)) break;
        s->address = 0;
        reply(s, NULL, 0, status_of(update_erase_all(link)));
        return;
    case PROG_MULTI: {
        const uint8_t n = s->arg[0];
        if (!ready(s) || n % 4U != 0U || update_owner() != link) break;
        const int r = update_write(link, s->address, &s->arg[1], n);
        if (r == MUBL_OK) s->address += n;
        reply(s, NULL, 0, status_of(r));
        return;
    }
    case GET_CRC: {
        uint32_t crc;
        if (!update_crc32(0, 0, MUBL_APP_SIZE, &crc)) break;
        reply_word(s, crc, OK);
        return;
    }
    case GET_OTP:
        reply_word(s, 0, OK);
        return;
    case GET_SN:
        if (word > 12U) break;
        hal_unique_id(uid);
        reply(s, &uid[word], 4, OK);
        return;
    case GET_CHIP:
        reply_word(s, hal_idcode(), OK);
        return;
    case GET_CHIP_DES:
        reply_string(s, MUBL_SERIAL_CHIP_DES);
        return;
    case GET_VERSION:
        reply_string(s, MUBL_BOARD_NAME);
        return;
    case BOOT: {
        int r = MUBL_OK;
        if (update_owner() == link) r = update_finish(link);
        if (r == MUBL_OK && mubl_app()->status != MUBL_APP_OK) r = MUBL_E_IMAGE;
        reply(s, NULL, 0, status_of(r));
        if (r == MUBL_OK) mubl_boot(20);
        return;
    }
    case SET_BAUD:
        if (!ready(s) || word < 1200U) break;
        reply(s, NULL, 0, OK);
#if MUBL_SERIAL
        if (link == MUBL_LINK_SERIAL) {          /* over USB the rate means nothing */
            uart_set_baud(link, word);
            mubl_session()->uart_baud = word;
        }
#endif
        return;
    default:
        break;
    }
    reply(s, NULL, 0, INVALID);
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

static void rx(serial_t* s, uint8_t b, uint32_t now)
{
    if (s->op != 0U && mubl_since(now, s->last) > (int32_t)COMMAND_TIMEOUT_MS) s->op = 0;
    s->last = now;
    if (s->op == 0U) {
        const int n = arguments(b);
        if (n < 0) return;
        s->op = b;
        s->need = (uint8_t)n;
        s->got = 0;
        return;
    }
    if (s->got < s->need) {
        s->arg[s->got++] = b;
        if (s->op == PROG_MULTI && s->got == 1U) {
            if (b > PROG_MAX) {
                s->op = 0;
                return;
            }
            s->need = (uint8_t)(1U + b);
        }
        return;
    }
    const uint8_t op = s->op;
    s->op = 0;
    if (b != EOC) {
        if (s->owner) reply(s, NULL, 0, INVALID);
        return;
    }
    /* A well-formed GET_SYNC claims the port; until then nothing else is answered. */
    if (!s->owner) {
        if (op != GET_SYNC) return;
#if MUBL_SERIAL
        if (s->link == MUBL_LINK_SERIAL && !uart_claim(s->link)) return;
#endif
        s->owner = 1;
        mubl_detected(s->link);
    }
    execute(s, op);
}

static void session_poll(serial_t* s, uint32_t now)
{
    if (update_owner() == s->link && mubl_since(now, s->last) > (int32_t)SESSION_TIMEOUT_MS) update_abort(s->link);
}

static void instance_start(serial_t* s, uint8_t link)
{
    memset(s, 0, sizeof *s);
    s->link = link;
}

#if MUBL_SERIAL
void serial_rx(uint8_t b, uint32_t now)
{
    rx(UART_INST, b, now);
}

void serial_poll(uint32_t now)
{
    session_poll(UART_INST, now);
}

void serial_start(void)
{
    instance_start(UART_INST, MUBL_LINK_SERIAL);
    const mubl_request_t* req = mubl_request();
    const uint32_t baud = req->link == MUBL_LINK_SERIAL && req->uart_baud != 0U ? req->uart_baud : MUBL_SERIAL_BAUD;
    uart_listen(MUBL_LINK_SERIAL, MUBL_SERIAL_PORT, baud);
    mubl_session()->uart_baud = baud;
}

void serial_stop(void)
{
    update_abort(MUBL_LINK_SERIAL);
    uart_unlisten(MUBL_LINK_SERIAL);
}
#endif

#if MUBL_USB
/*
 * USB: the same protocol over CDC-ACM.  A host is present while start-of-frame packets arrive;
 * with MUBL_USB_STAY that keeps the bootloader running (see mubl_config.h).  Received bytes are
 * taken only as fast as the protocol consumes them, so the host is held off while flash is busy.
 */
static struct {
    uint8_t host;             /* frames seen within MUBL_USB_GONE_MS */
    uint32_t last_sof;
} usb;

void usb_start(void)
{
    instance_start(USB_INST, MUBL_LINK_USB);
    memset(&usb, 0, sizeof usb);
    hal_usb_start();
}

void usb_poll(uint32_t now)
{
    const uint8_t ev = hal_usb_poll();
    if (ev & (HAL_USB_SOF | HAL_USB_RESET)) {
        usb.last_sof = now;
        if (!usb.host) {
            usb.host = 1;
            mubl_detected(MUBL_LINK_USB);
            if (MUBL_USB_STAY) mubl_usb_host(true);
        }
    } else if (usb.host && mubl_since(now, usb.last_sof) > (int32_t)MUBL_USB_GONE_MS) {
        usb.host = 0;
        if (MUBL_USB_STAY) mubl_usb_host(false);
    }
    int c;
    while ((c = hal_usb_getc()) >= 0) rx(USB_INST, (uint8_t)c, now);
    session_poll(USB_INST, now);
}

void usb_stop(void)
{
    update_abort(MUBL_LINK_USB);
    hal_usb_stop();
}
#endif

#endif
