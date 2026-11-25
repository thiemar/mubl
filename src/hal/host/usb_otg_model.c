/* Host-side model of the USB OTG core in device mode.  See usb_otg_model.h. */

#include <stdio.h>
#include <string.h>

#include "usb_otg_model.h"

#if MUBL_USB_OTG_CORE

#ifndef STM32_OTG_VBDEN
#define STM32_OTG_VBDEN 1              /* the F446/H7 core; 0: the F405 core */
#endif
#ifndef OTGM_FIFO_WORDS
#define OTGM_FIFO_WORDS 320U           /* F4 OTG_FS; H7: 1024 */
#endif

/* Registers (RM0090 35.12, RM0433 57.14) */
#define GOTGCTL 0x000u
#define GUSBCFG 0x00Cu
#define GRSTCTL 0x010u
#define GINTSTS 0x014u
#define GRXSTSR 0x01Cu
#define GRXSTSP 0x020u
#define GRXFSIZ 0x024u
#define DIEPTXF0 0x028u
#define GCCFG 0x038u
#define DCFG 0x800u
#define DCTL 0x804u
#define DSTS 0x808u

#define NEP 4u
#define MAX_PACKET 64u
#define RXQ 32u

static uint32_t regs[0x1000u / 4u];   /* plain storage for what has no behaviour here */

static struct ep {
    uint32_t ctl;                       /* MPSIZ, USBAEP, EPTYP, STALL, TXFNUM as written */
    uint32_t intr;
    uint32_t tsiz;
    uint8_t enabled, nak, dpid;
    uint8_t fifo[1024];                 /* IN: transmit FIFO contents */
    uint16_t fifo_n;
} in_ep[NEP], out_ep[NEP];

static struct {
    uint32_t status;
    uint8_t data[MAX_PACKET];
} rxq[RXQ];
static uint32_t rx_head, rx_n, rx_bytes;
static const uint8_t *cursor;          /* data of the entry last popped */
static uint16_t cursor_left;

static uint32_t events;                 /* GINTSTS SOF, USBRST, ENUMDNE */
static uint8_t dev_addr;                /* the address in use: a new DAD takes effect after the status stage */
static bool powered, enumerated, gonak, was_connected, detached;
static uint8_t host_in_toggle[NEP], host_out_toggle[NEP];
static int errors;
static bool quiet;

#define FAIL(...)                         \
    do {                                  \
        if (!quiet) {                     \
            fprintf(stderr, "  otg: ");   \
            fprintf(stderr, __VA_ARGS__); \
            fputc('\n', stderr);          \
        }                                 \
        errors++;                         \
    } while (0)

/* Bits */
#define CTL_MPSIZ 0x7FFu
#define CTL_USBAEP (1u << 15)
#define CTL_DPID (1u << 16)
#define CTL_NAKSTS (1u << 17)
#define CTL_EPTYP (3u << 18)
#define CTL_STALL (1u << 21)
#define CTL_TXFNUM (0xFu << 22)
#define CTL_CNAK (1u << 26)
#define CTL_SNAK (1u << 27)
#define CTL_SD0PID (1u << 28)
#define CTL_EPDIS (1u << 30)
#define CTL_EPENA (1u << 31)
#define INT_XFRC 1u
#define INT_EPDISD 2u
#define INT_STUP 8u
#define INT_INEPNE 0x40u

static uint32_t reg(uint32_t off) { return regs[off / 4u]; }

static bool session_valid(void) {
#if STM32_OTG_VBDEN
    return !(reg(GCCFG) & (1u << 21)) && (reg(GOTGCTL) & (3u << 6)) == (3u << 6);
#else
    return (reg(GCCFG) & (1u << 21)) != 0;      /* NOVBUSSENS */
#endif
}

static bool connected(void) {
    return powered && (reg(GCCFG) & (1u << 16)) && (reg(GUSBCFG) & (1u << 30)) &&
           !(reg(DCTL) & 2u) && session_valid();
}

static void track_connection(void) {
    const bool c = connected();
    if (was_connected && !c) {
        detached = true;
        enumerated = false;
    }
    was_connected = c;
}

static uint16_t mps(const struct ep *e, uint8_t n) {
    if (n == 0) {
        static const uint16_t ep0[4] = {64, 32, 16, 8};
        return ep0[e->ctl & 3u];
    }
    return (uint16_t)(e->ctl & CTL_MPSIZ);
}

/* Transmit FIFO n: start and depth in words */
static void txf(uint8_t n, uint32_t *start, uint32_t *depth) {
    const uint32_t v = n == 0 ? reg(DIEPTXF0) : reg(0x104u + 4u * (n - 1u));
    *start = v & 0xFFFFu;
    *depth = v >> 16;
}

static void check_fifos(uint8_t n) {
    uint32_t s, d, s2, d2;
    const uint32_t rx = reg(GRXFSIZ) & 0xFFFFu;
    txf(n, &s, &d);
    if (rx < 16u || d < 16u || s < rx || s + d > OTGM_FIFO_WORDS) {
        FAIL("transmit FIFO %u (%u words at %u) or receive FIFO (%u words) misplaced", n, d, s, rx);
        return;
    }
    for (uint8_t k = 0; k < NEP; k++) {
        if (k == n) continue;
        txf(k, &s2, &d2);
        if (d2 != 0 && s < s2 + d2 && s2 < s + d) FAIL("transmit FIFOs %u and %u overlap", n, k);
    }
}

static void core_reset(void) {
    memset(in_ep, 0, sizeof(in_ep));
    memset(out_ep, 0, sizeof(out_ep));
    for (uint8_t i = 0; i < NEP; i++) {
        in_ep[i].nak = out_ep[i].nak = 1;
    }
    rx_n = rx_head = rx_bytes = 0;
    cursor_left = 0;
    events = 0;
    gonak = false;
    regs[DCFG / 4u] = 0;
    regs[DCTL / 4u] = 2u;                        /* soft-disconnected */
}

/* ---- Driver side ------------------------------------------------------------------------ */

bool hw_start(void) {
    memset(regs, 0, sizeof(regs));
    core_reset();
    regs[GRSTCTL / 4u] = 1u << 31;               /* AHB idle */
    regs[GINTSTS / 4u] = 0;
    powered = true;
    enumerated = false;
    return true;
}

void hw_stop(void) {
    track_connection();
    powered = false;
    track_connection();
    core_reset();
}

static uint32_t ctl_read(const struct ep *e) {
    return e->ctl | (e->enabled ? CTL_EPENA : 0u) | (e->nak ? CTL_NAKSTS : 0u) | (e->dpid ? CTL_DPID : 0u);
}

static void ctl_write(struct ep *e, uint8_t n, bool in, uint32_t v) {
    const uint32_t rw = CTL_MPSIZ | CTL_USBAEP | CTL_EPTYP | (in ? CTL_TXFNUM : 0u);
    e->ctl = (e->ctl & ~rw) | (v & rw);
    if (n == 0) {
        e->ctl |= CTL_USBAEP;
        if (v & CTL_STALL) e->ctl |= CTL_STALL;  /* EP0: set only; a SETUP clears it */
    } else {
        e->ctl = (e->ctl & ~CTL_STALL) | (v & CTL_STALL);
    }
    if (v & CTL_CNAK) e->nak = 0;
    if (v & CTL_SNAK) {
        e->nak = 1;
        if (in) e->intr |= INT_INEPNE;
    }
    if (v & CTL_SD0PID) e->dpid = 0;
    if ((v & CTL_EPDIS) && e->enabled) {
        if (!in && !gonak) FAIL("OUT endpoint %u disabled without the global OUT NAK", n);
        if (in && !(e->intr & INT_INEPNE)) FAIL("IN endpoint %u disabled before its NAK took effect", n);
        e->enabled = 0;
        e->intr |= INT_EPDISD;
    }
    if ((v & CTL_EPENA) && !(v & CTL_EPDIS) && !e->enabled) {
        if (!(e->ctl & CTL_USBAEP)) FAIL("%s endpoint %u enabled while not active", in ? "IN" : "OUT", n);
        if (in) check_fifos((uint8_t)((e->ctl & CTL_TXFNUM) >> 22));
        if (in && ((e->ctl & CTL_TXFNUM) >> 22) != n) FAIL("IN endpoint %u uses FIFO %u", n, (e->ctl & CTL_TXFNUM) >> 22);
        e->enabled = 1;
    }
}

static int fifo_ep(uint32_t off) {
    return off >= 0x1000u && off < 0x1000u + 0x1000u * NEP ? (int)(off / 0x1000u) - 1 : -1;
}

uint32_t otgm_rd(uint32_t off) {
    if (!powered) {
        return 0;
    }
    const int f = fifo_ep(off);
    if (f >= 0) {
        uint32_t w = 0;
        if (cursor_left == 0) {
            FAIL("receive FIFO read with no data left in the popped entry");
            return 0;
        }
        for (uint8_t k = 0; k < 4 && cursor_left; k++, cursor_left--) {
            w |= (uint32_t)*cursor++ << (8u * k);
        }
        return w;
    }
    if (off >= 0x900u && off < 0x900u + 0x20u * NEP) {
        struct ep *e = &in_ep[(off - 0x900u) / 0x20u];
        switch (off & 0x1Fu) {
            case 0x00: return ctl_read(e);
            case 0x08: return e->intr;
            case 0x10: return e->tsiz;
            case 0x18: {
                uint32_t s, d;
                txf((uint8_t)((off - 0x900u) / 0x20u), &s, &d);
                return d - (e->fifo_n + 3u) / 4u;
            }
            default: return 0;
        }
    }
    if (off >= 0xB00u && off < 0xB00u + 0x20u * NEP) {
        struct ep *e = &out_ep[(off - 0xB00u) / 0x20u];
        switch (off & 0x1Fu) {
            case 0x00: return ctl_read(e);
            case 0x08: return e->intr;
            case 0x10: return e->tsiz;
            default: return 0;
        }
    }
    switch (off) {
        case GINTSTS:
            return events | (rx_n ? (1u << 4) : 0u) | (gonak ? (1u << 7) : 0u) |
                   ((reg(GUSBCFG) & (1u << 30)) ? 0u : 1u);
        case GRXSTSP: {
            if (rx_n == 0) {
                FAIL("GRXSTSP read with the receive FIFO empty");
                return 0;
            }
            if (cursor_left) FAIL("receive FIFO entry popped before its data was read");
            const uint32_t st = rxq[rx_head].status;
            const uint8_t ep = st & 0xFu, kind = (st >> 17) & 0xFu;
            cursor = rxq[rx_head].data;
            cursor_left = (st >> 4) & 0x7FFu;
            rx_bytes -= cursor_left;
            rx_head = (rx_head + 1u) % RXQ;
            rx_n--;
            if (kind == 4) out_ep[0].intr |= INT_STUP;   /* SETUP stage done */
            if (kind == 3) out_ep[ep].intr |= INT_XFRC;  /* OUT transfer completed */
            return st;
        }
        case GRSTCTL:
            return 1u << 31;
        default:
            return reg(off);
    }
}

void otgm_wr(uint32_t off, uint32_t v) {
    if (!powered) {
        FAIL("write to 0x%03X with the core's clock off", off);
        return;
    }
    const int f = fifo_ep(off);
    if (f >= 0) {
        struct ep *e = &in_ep[f];
        uint32_t s, d;
        txf((uint8_t)f, &s, &d);
        if ((e->fifo_n + 4u) / 4u > d || e->fifo_n + 4u > sizeof(e->fifo)) {
            FAIL("transmit FIFO %d overflow", f);
            return;
        }
        for (uint8_t k = 0; k < 4; k++) e->fifo[e->fifo_n++] = (uint8_t)(v >> (8u * k));
        return;
    }
    if (off >= 0x900u && off < 0x900u + 0x20u * NEP) {
        const uint8_t n = (uint8_t)((off - 0x900u) / 0x20u);
        switch (off & 0x1Fu) {
            case 0x00: ctl_write(&in_ep[n], n, true, v); break;
            case 0x08: in_ep[n].intr &= ~v; break;
            case 0x10: in_ep[n].tsiz = v; break;
            default: break;
        }
        return;
    }
    if (off >= 0xB00u && off < 0xB00u + 0x20u * NEP) {
        const uint8_t n = (uint8_t)((off - 0xB00u) / 0x20u);
        switch (off & 0x1Fu) {
            case 0x00: ctl_write(&out_ep[n], n, false, v); break;
            case 0x08: out_ep[n].intr &= ~v; break;
            case 0x10: out_ep[n].tsiz = v; break;
            default: break;
        }
        return;
    }
    switch (off) {
        case GINTSTS:
            events &= ~(v & ((1u << 3) | (1u << 12) | (1u << 13)));
            return;
        case GRSTCTL:
            if (v & 1u) {                          /* core soft reset */
                core_reset();
                enumerated = false;
            }
            if (v & (1u << 4)) {                   /* receive FIFO flush */
                rx_n = rx_head = rx_bytes = 0;
                cursor_left = 0;
            }
            if (v & (1u << 5)) {                   /* transmit FIFO flush */
                const uint32_t n = (v >> 6) & 0x1Fu;
                for (uint8_t k = 0; k < NEP; k++) {
                    if (n == 0x10u || n == k) {
                        if (n != 0x10u && in_ep[k].enabled) FAIL("transmit FIFO %u flushed while its endpoint is enabled", k);
                        in_ep[k].fifo_n = 0;
                    }
                }
            }
            break;
        case DCTL:
            regs[DCTL / 4u] = v & 0x3u;
            if (v & (1u << 9)) gonak = true;
            if (v & (1u << 10)) gonak = false;
            track_connection();
            return;
        default:
            regs[off / 4u] = v;
            track_connection();
            return;
    }
}

/* ---- Host side ---------------------------------------------------------------------------- */

static bool addressed(uint8_t address) {
    return connected() && enumerated && dev_addr == address;
}

static bool rx_push(uint32_t status, const uint8_t *data, uint16_t n) {
    const uint32_t words = (reg(GRXFSIZ) & 0xFFFFu);
    if (rx_n == RXQ || (rx_bytes + n + 3u) / 4u + rx_n + 1u > words) {
        return false;
    }
    const uint32_t i = (rx_head + rx_n) % RXQ;
    rxq[i].status = status | ((uint32_t)n << 4);
    if (n) memcpy(rxq[i].data, data, n);
    rx_n++;
    rx_bytes += n;
    return true;
}

void usbm_reset_all(void) {
    memset(regs, 0, sizeof(regs));
    core_reset();
    powered = enumerated = was_connected = detached = false;
    dev_addr = 0;
    memset(host_in_toggle, 0, sizeof(host_in_toggle));
    memset(host_out_toggle, 0, sizeof(host_out_toggle));
    errors = 0;
}

void usbm_bus_reset(void) {
    memset(host_in_toggle, 0, sizeof(host_in_toggle));
    memset(host_out_toggle, 0, sizeof(host_out_toggle));
    if (!connected()) {
        return;
    }
    for (uint8_t i = 0; i < NEP; i++) {
        in_ep[i].enabled = out_ep[i].enabled = 0;
        in_ep[i].nak = out_ep[i].nak = 1;
        in_ep[i].dpid = out_ep[i].dpid = 0;
        if (i) {
            in_ep[i].ctl &= ~CTL_USBAEP;
            out_ep[i].ctl &= ~CTL_USBAEP;
        }
    }
    events |= (1u << 12) | (1u << 13);          /* USBRST, then ENUMDNE once the reset ends */
    enumerated = true;
    dev_addr = 0;
}

void usbm_sof(void) {
    if (connected()) events |= 1u << 3;
}

usbm_handshake_t usbm_setup(uint8_t address, const uint8_t packet[8]) {
    struct ep *o = &out_ep[0];
    if (!addressed(address)) return USBM_TIMEOUT;
    const uint32_t stupcnt = (o->tsiz >> 29) & 3u;
    if (stupcnt == 0) {
        FAIL("SETUP received with no SETUP packets left in DOEPTSIZ0");
    }
    if (!rx_push((6u << 17), packet, 8) || !rx_push((4u << 17), NULL, 0)) {
        return USBM_TIMEOUT;                     /* no room: the host retries */
    }
    o->tsiz = (o->tsiz & ~(3u << 29)) | ((stupcnt ? stupcnt - 1u : 0u) << 29);
    in_ep[0].ctl &= ~CTL_STALL;
    o->ctl &= ~CTL_STALL;
    in_ep[0].nak = o->nak = 1;
    in_ep[0].dpid = o->dpid = 1;                 /* data and status stages start with DATA1 */
    host_in_toggle[0] = host_out_toggle[0] = 1;
    return USBM_ACK;
}

static bool active(uint8_t ep, const struct ep *e) {
    return ep < NEP && (ep == 0 || (e->ctl & CTL_USBAEP));
}

usbm_handshake_t usbm_out(uint8_t address, uint8_t ep, const uint8_t *data, uint16_t len) {
    struct ep *e = &out_ep[ep < NEP ? ep : 0];
    if (!addressed(address) || !active(ep, e)) return USBM_TIMEOUT;
    if (e->ctl & CTL_STALL) return USBM_STALL;
    if (!e->enabled || e->nak || gonak) return USBM_NAK;
    if (len > mps(e, ep) || len > (e->tsiz & 0x7FFFFu)) {
        FAIL("OUT of %u bytes on endpoint %u, which expects at most %u", len, ep, (unsigned)(e->tsiz & 0x7FFFFu));
        return USBM_TIMEOUT;
    }
    if (e->dpid != host_out_toggle[ep]) {
        FAIL("endpoint %u OUT data toggle: host sent DATA%u, device expects DATA%u", ep, host_out_toggle[ep], e->dpid);
        host_out_toggle[ep] ^= 1u;
        return USBM_ACK;
    }
    if (!rx_push((2u << 17) | ep | ((uint32_t)e->dpid << 15), data, len)) return USBM_NAK;
    e->dpid ^= 1u;
    host_out_toggle[ep] ^= 1u;
    const uint32_t pkts = ((e->tsiz >> 19) & 0x3FFu) - 1u;
    e->tsiz = (e->tsiz & 0xE0000000u) | (pkts << 19) | ((e->tsiz & 0x7FFFFu) - len);
    if (pkts == 0 || len < mps(e, ep)) {
        (void)rx_push((3u << 17) | ep, NULL, 0);
        e->enabled = 0;                          /* NAKs until enabled again */
        e->nak = 1;
    }
    return USBM_ACK;
}

usbm_handshake_t usbm_in(uint8_t address, uint8_t ep, uint8_t *data, uint16_t *len) {
    struct ep *e = &in_ep[ep < NEP ? ep : 0];
    *len = 0;
    if (!addressed(address) || !active(ep, e)) return USBM_TIMEOUT;
    if (e->ctl & CTL_STALL) return USBM_STALL;
    if (!e->enabled || e->nak) return USBM_NAK;
    const uint32_t left = e->tsiz & (ep == 0 ? 0x7Fu : 0x7FFFFu);
    const uint16_t n = (uint16_t)(left < mps(e, ep) ? left : mps(e, ep));
    if (e->fifo_n < n) return USBM_NAK;          /* the data is not in the FIFO yet */
    if (e->dpid != host_in_toggle[ep]) {
        FAIL("endpoint %u IN data toggle: device sent DATA%u, host expects DATA%u", ep, e->dpid, host_in_toggle[ep]);
    }
    memcpy(data, e->fifo, n);
    *len = n;
    const uint16_t pad = (uint16_t)((n + 3u) & ~3u);  /* FIFO words */
    memmove(e->fifo, e->fifo + pad, (size_t)(e->fifo_n - (pad < e->fifo_n ? pad : e->fifo_n)));
    e->fifo_n = (uint16_t)(e->fifo_n > pad ? e->fifo_n - pad : 0u);
    const uint32_t pkt_mask = ep == 0 ? 3u : 0x3FFu;
    const uint32_t pkts = ((e->tsiz >> 19) & pkt_mask) - 1u;
    e->tsiz = (e->tsiz & ~((pkt_mask << 19) | (ep == 0 ? 0x7Fu : 0x7FFFFu))) | (pkts << 19) | (left - n);
    e->dpid ^= 1u;
    host_in_toggle[ep] ^= 1u;
    if (pkts == 0) {
        e->enabled = 0;
        e->intr |= INT_XFRC;
        if (ep == 0) dev_addr = (uint8_t)((reg(DCFG) >> 4) & 0x7Fu);   /* after a status stage */
        if (e->fifo_n) FAIL("endpoint %u finished with %u bytes left in its FIFO", ep, e->fifo_n);
    }
    return USBM_ACK;
}

void usbm_reset_host_toggle(uint8_t ep_address) {
    if (ep_address & 0x80u) {
        host_in_toggle[ep_address & 0x3u] = 0;
    } else {
        host_out_toggle[ep_address & 0x3u] = 0;
    }
}

bool usbm_connected(void) {
    return connected();
}

bool usbm_take_detach(void) {
    track_connection();
    const bool d = detached;
    detached = false;
    return d;
}

uint8_t usbm_daddr(void) {
    return (uint8_t)(0x80u | ((reg(DCFG) >> 4) & 0x7Fu));
}

int usbm_take_errors(void) {
    const int n = errors;
    errors = 0;
    return n;
}

void usbm_set_quiet(bool q) {
    quiet = q;
}

#endif
