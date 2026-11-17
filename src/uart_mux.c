/*
 * UART port sharing between the serial protocol and SRXL2 (see links.h).
 */
#include "hal/hal.h"
#include "links.h"

#if MUBL_SERIAL || MUBL_SRXL2

typedef struct {
    uint8_t port;
    uint8_t listening;
    uint32_t baud;
} listener_t;

static struct {
    listener_t l[MUBL_LINK_COUNT];        /* by link ID */
    uint8_t owner[2];                     /* link that claimed each port */
    uint8_t current[2];                   /* link whose baud rate the port runs at; 0: stopped */
    uint32_t slice_end[2];
} x;

static void deliver(uint8_t link, uint8_t byte, uint32_t now)
{
    switch (link) {
#if MUBL_SERIAL
    case MUBL_LINK_SERIAL: serial_rx(byte, now); break;
#endif
#if MUBL_SRXL2
    case MUBL_LINK_SRXL2: srxl2_rx(byte, now); break;
#endif
    default: break;
    }
}

/* Highest-priority listener on `port` whose rate is not `after`'s, after `after` in rank order
 * (wrapping).  0 if none. */
static uint8_t next_rate(uint8_t port, uint8_t after)
{
    uint8_t best = 0, first = 0;
    for (uint8_t id = 1; id < MUBL_LINK_COUNT; id++) {
        const listener_t* l = &x.l[id];
        if (!l->listening || l->port != port) continue;
        if (after != 0U && l->baud == x.l[after].baud) continue;
        if (first == 0U || mubl_rank(id) < mubl_rank(first)) first = id;
        if (after != 0U && mubl_rank(id) > mubl_rank(after) && (best == 0U || mubl_rank(id) < mubl_rank(best))) {
            best = id;
        }
    }
    return best != 0U ? best : first;
}

static void switch_to(uint8_t port, uint8_t link, uint32_t now)
{
    x.current[port] = link;
    x.slice_end[port] = now + MUBL_UART_SLICE_MS;
    hal_uart_start(port, x.l[link].baud);
}

void uart_mux_init(void)
{
    memset(&x, 0, sizeof x);
}

void uart_listen(uint8_t link, uint8_t port, uint32_t baud)
{
    x.l[link].port = port;
    x.l[link].baud = baud;
    x.l[link].listening = 1;
    if (x.current[port] == 0U) {
        switch_to(port, link, mubl_now());
    } else if (x.owner[port] == 0U && mubl_rank(link) < mubl_rank(x.current[port]) &&
               baud != x.l[x.current[port]].baud) {
        switch_to(port, link, mubl_now());       /* a higher-priority rate is tried first */
    }
}

void uart_unlisten(uint8_t link)
{
    listener_t* l = &x.l[link];
    if (!l->listening) return;
    l->listening = 0;
    const uint8_t port = l->port;
    if (x.owner[port] == link) x.owner[port] = 0;
    if (x.current[port] != link) return;
    const uint8_t next = next_rate(port, 0);
    if (next == 0U) {
        x.current[port] = 0;
        hal_uart_stop(port);
    } else if (x.l[next].baud != l->baud) {
        switch_to(port, next, mubl_now());
    } else {
        x.current[port] = next;
    }
}

void uart_mux_poll(uint32_t now)
{
    for (uint8_t port = 0; port < 2U; port++) {
        const uint8_t cur = x.current[port];
        if (cur == 0U) continue;
        int c;
        while ((c = hal_uart_getc(port)) >= 0) {
            if (x.owner[port] != 0U) {
                deliver(x.owner[port], (uint8_t)c, now);
                continue;
            }
            /* Every listener at the current rate sees the byte; a claim mid-loop takes effect for
             * the next byte. */
            for (uint8_t id = 1; id < MUBL_LINK_COUNT && x.owner[port] == 0U; id++) {
                const listener_t* l = &x.l[id];
                if (l->listening && l->port == port && l->baud == x.l[x.current[port]].baud) {
                    deliver(id, (uint8_t)c, now);
                }
            }
        }
        if (x.owner[port] == 0U && (int32_t)(now - x.slice_end[port]) >= 0) {
            const uint8_t next = next_rate(port, x.current[port]);
            if (next != 0U && x.l[next].baud != x.l[x.current[port]].baud) {
                switch_to(port, next, now);
            } else {
                x.slice_end[port] = now + MUBL_UART_SLICE_MS;
            }
        }
    }
}

bool uart_claim(uint8_t link)
{
    const uint8_t port = x.l[link].port;
    if (!x.l[link].listening) return false;
    if (x.owner[port] != 0U) return x.owner[port] == link;
    x.owner[port] = link;
    x.current[port] = link;
    return true;
}

bool uart_may_send(uint8_t link)
{
    const listener_t* l = &x.l[link];
    const uint8_t port = l->port;
    if (!l->listening) return false;
    if (x.owner[port] != 0U) return x.owner[port] == link;
    if (l->baud != x.l[x.current[port]].baud) return false;
    for (uint8_t id = 1; id < MUBL_LINK_COUNT; id++) {
        if (x.l[id].listening && x.l[id].port == port && mubl_rank(id) < mubl_rank(link)) return false;
    }
    return true;
}

void uart_send(uint8_t link, const uint8_t* data, uint16_t n)
{
    if (uart_may_send(link)) hal_uart_write(x.l[link].port, data, n);
}

void uart_set_baud(uint8_t link, uint32_t baud)
{
    const uint8_t port = x.l[link].port;
    if (x.owner[port] != link || x.l[link].baud == baud) return;
    x.l[link].baud = baud;
    hal_uart_start(port, baud);
}

uint32_t uart_baud(uint8_t link)
{
    return x.l[link].baud;
}

#endif
