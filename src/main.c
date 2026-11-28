/*
 * µBL boot manager.
 *
 * After reset: read the handoff, check the application, decide whether to stay, start every link
 * the board has (or only the one the application named), then poll them in priority order.
 *
 * With a valid application and no reason to stay, the application starts MUBL_BOOT_DELAY_MS
 * after reset unless a host has started using a link by then (mubl_hold) or an update is running.
 * The bootloader stays when there is no valid application, when the application asked it to, when
 * the hold pin is asserted, when the application failed to confirm MUBL_MAX_BOOT_ATTEMPTS
 * boots in a row, or while a USB host is connected (MUBL_USB_STAY).
 *
 * Link priority (MUBL_LINK_PRIORITY, highest first) decides the order links are started and
 * polled, which link may speak first on a shared UART, and with MUBL_LINK_EXCLUSIVE, which links
 * are dropped once a higher-priority link has found a peer.
 */
#include "handoff.h"
#include "hal/hal.h"
#include "image.h"
#include "links.h"
#include "update.h"

enum { ACTION_NONE = 0, ACTION_BOOT = 1, ACTION_RESTART = 2 };

static const uint8_t priority[] = {MUBL_LINK_PRIORITY};

static struct {
    mubl_app_t app;
    mubl_request_t req;
    mubl_session_t session;
    uint32_t deadline;        /* start the application then, unless staying */
    uint32_t action_at;
    uint32_t rng;
    uint32_t led_at;
    uint8_t stay;             /* MUBL_STAY_* */
    uint8_t running;          /* links started, by mask */
    uint8_t detected;
    uint8_t action;
    uint8_t extended;
    uint8_t led;
} m;

uint8_t mubl_rank(uint8_t link)
{
    for (uint8_t i = 0; i < sizeof priority; i++) {
        if (priority[i] == link) return i;
    }
    return 0xFF;
}

static void link_start(uint8_t id)
{
    switch (id) {
#if MUBL_DRONECAN
    case MUBL_LINK_DRONECAN: dronecan_start(); break;
#endif
#if MUBL_SERIAL
    case MUBL_LINK_SERIAL: serial_start(); break;
#endif
#if MUBL_SRXL2
    case MUBL_LINK_SRXL2: srxl2_start(); break;
#endif
#if MUBL_USB
    case MUBL_LINK_USB: usb_start(); break;
#endif
    default: return;
    }
    m.running |= (uint8_t)(1U << id);
}

static void link_stop(uint8_t id)
{
    if (!(m.running & (1U << id))) return;
    m.running &= (uint8_t)~(1U << id);
    switch (id) {
#if MUBL_DRONECAN
    case MUBL_LINK_DRONECAN: dronecan_stop(); break;
#endif
#if MUBL_SERIAL
    case MUBL_LINK_SERIAL: serial_stop(); break;
#endif
#if MUBL_SRXL2
    case MUBL_LINK_SRXL2: srxl2_stop(); break;
#endif
#if MUBL_USB
    case MUBL_LINK_USB: usb_stop(); break;
#endif
    default: break;
    }
}

static void link_poll(uint8_t id, uint32_t now)
{
    switch (id) {
#if MUBL_DRONECAN
    case MUBL_LINK_DRONECAN: dronecan_poll(now); break;
#endif
#if MUBL_SERIAL
    case MUBL_LINK_SERIAL: serial_poll(now); break;
#endif
#if MUBL_SRXL2
    case MUBL_LINK_SRXL2: srxl2_poll(now); break;
#endif
#if MUBL_USB
    case MUBL_LINK_USB: usb_poll(now); break;
#endif
    default: break;
    }
}

static void start_app(void)
{
    if (m.app.status != MUBL_APP_OK) {
        if (m.stay == MUBL_STAY_NO) m.stay = MUBL_STAY_NO_APP;
        return;
    }
    for (uint8_t i = 0; i < sizeof priority; i++) link_stop(priority[i]);
    /* Before hal_deinit: the block's CRC may come from the CRC unit, which that stops. */
    handoff_write(&m.session, (uint8_t)(m.req.boot_attempts + 1U));
    hal_deinit();
    hal_start_app();
}

void mubl_init(void)
{
    memset(&m, 0, sizeof m);
    hal_init();
    handoff_read(&m.req);
    update_init();
    image_check(&m.app, NULL);

    uint8_t uid[16];
    hal_unique_id(uid);
    m.rng = 0x9E3779B9U;
    for (uint8_t i = 0; i < sizeof uid; i++) m.rng = (m.rng ^ uid[i]) * 0x01000193U;

    if (m.app.status != MUBL_APP_OK) {
        m.stay = MUBL_STAY_NO_APP;
    } else if (m.req.stay) {
        m.stay = MUBL_STAY_REQUESTED;
    } else if (hal_hold_pin()) {
        m.stay = MUBL_STAY_HOLD_PIN;
#if MUBL_MAX_BOOT_ATTEMPTS
    } else if (m.req.boot_attempts >= MUBL_MAX_BOOT_ATTEMPTS) {
        m.stay = MUBL_STAY_BOOT_LOOP;
#endif
    }
    m.deadline = hal_millis() + MUBL_BOOT_DELAY_MS;
    hal_led(true);

#if MUBL_SERIAL || MUBL_SRXL2
    uart_mux_init();
#endif
    uint8_t named = 0;
    for (uint8_t i = 0; i < sizeof priority; i++) named |= m.req.link == priority[i];
    for (uint8_t i = 0; i < sizeof priority; i++) {
        if (!named || m.req.link == priority[i]) link_start(priority[i]);
    }
}

void mubl_step(void)
{
    hal_watchdog_refresh();
#if MUBL_SERIAL || MUBL_SRXL2
    uart_mux_poll(hal_millis());
#endif
    /* Each link reads the clock afresh: received bytes or an earlier link may have blocked (replies,
     * flash), and a link compares the time with timestamps it took meanwhile. */
    for (uint8_t i = 0; i < sizeof priority; i++) {
        if (m.running & (1U << priority[i])) link_poll(priority[i], hal_millis());
    }
    const uint32_t now = hal_millis();
    if (m.action != ACTION_NONE && (int32_t)(now - m.action_at) >= 0 && !update_active()) {
        if (m.action == ACTION_RESTART) {
            for (uint8_t i = 0; i < sizeof priority; i++) link_stop(priority[i]);
            hal_reset();
        }
        m.action = ACTION_NONE;
        start_app();
    }
    if (m.stay == MUBL_STAY_NO && !update_active() && (int32_t)(now - m.deadline) >= 0) start_app();
}

uint32_t mubl_now(void)
{
    return hal_millis();
}

void mubl_detected(uint8_t link)
{
    if (m.detected & (1U << link)) return;
    m.detected |= (uint8_t)(1U << link);
    if (m.session.link == 0U) m.session.link = link;
    if (MUBL_DETECT_EXTEND_MS != 0 && !m.extended) {
        m.extended = 1;
        m.deadline += MUBL_DETECT_EXTEND_MS;
    }
#if MUBL_LINK_EXCLUSIVE
    for (uint8_t i = mubl_rank(link) + 1U; i < sizeof priority; i++) link_stop(priority[i]);
#endif
}

void mubl_hold(uint8_t link)
{
    m.session.link = link;
    if (m.stay == MUBL_STAY_NO || m.stay == MUBL_STAY_ALLOC) m.stay = MUBL_STAY_HOST;
}

void mubl_usb_host(bool present)
{
    if (present && m.stay == MUBL_STAY_NO) {
        m.stay = MUBL_STAY_USB;
    } else if (!present && m.stay == MUBL_STAY_USB) {
        m.stay = MUBL_STAY_NO;               /* boots now if the boot delay has passed */
    }
}

void mubl_wait_alloc(bool waiting)
{
    if (waiting && m.stay == MUBL_STAY_NO) {
        m.stay = MUBL_STAY_ALLOC;
    } else if (!waiting && m.stay == MUBL_STAY_ALLOC) {
        m.stay = MUBL_STAY_NO;
        m.deadline = hal_millis() + MUBL_BOOT_DELAY_MS;
    }
}

void mubl_boot(uint32_t delay_ms)
{
    m.action = ACTION_BOOT;
    m.action_at = hal_millis() + delay_ms;
}

void mubl_restart(uint32_t delay_ms)
{
    m.action = ACTION_RESTART;
    m.action_at = hal_millis() + delay_ms;
}

const mubl_app_t* mubl_app(void)
{
    return &m.app;
}

void mubl_set_app(const mubl_app_t* app)
{
    m.app = *app;
}

uint8_t mubl_stay_reason(void)
{
    return m.stay;
}

const mubl_request_t* mubl_request(void)
{
    return &m.req;
}

mubl_session_t* mubl_session(void)
{
    return &m.session;
}

uint32_t mubl_random(void)
{
    /* xorshift32 on a seed from the unique ID, stirred by the clock */
    uint32_t x = m.rng ^ hal_millis();
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    m.rng = x;
    return x;
}

void mubl_indicate(uint8_t activity)
{
    const uint32_t now = hal_millis();
    if (!activity) {
        hal_led(true);
    } else if ((int32_t)(now - m.led_at) >= 0) {
        m.led_at = now + 50U;
        m.led ^= 1U;
        hal_led(m.led != 0U);
    }
}

#ifndef MUBL_SIM
int main(void)
{
    mubl_init();
    for (;;) mubl_step();
}
#endif
