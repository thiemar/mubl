/*
 * The USB OTG core in device mode (Synopsys DWC2: OTG_FS on F4, OTG_HS or OTG_FS with the embedded
 * full-speed PHY on H7): the device layer of the CDC-ACM port (usb_cdc.h), polled.
 *
 *  - hal_usb_poll() is the only code that touches the core; its interrupt stays disabled (the
 *    status bits are set whatever the masks).
 *  - Received packets are read from the shared receive FIFO as their status entries are popped;
 *    SETUP and transfer completion are then acted on from the endpoint interrupt registers.
 *  - An OUT endpoint disables itself after each packet (it NAKs until re-enabled), so bulk OUT is
 *    re-armed only once the previous packet has been read: the host is held off while busy.
 *  - Endpoint control registers are written with EPENA, EPDIS, CNAK, SNAK and SD0PID clear except
 *    where meant (EPENA written as 0 has no effect, so a transfer the core finishes between the
 *    read and the write is not restarted).
 *  - One IN packet at a time per endpoint, written to its FIFO after the endpoint is enabled.
 *
 * FIFO memory (words): receive 128, EP0 IN 16, EP1 IN 16, EP3 IN 16: 176 of the 320 the F4's
 * OTG_FS has (H7: 1024).
 */
#include "usb_cdc.h"
#include "usb_otg_regs.h"

#include "hal/hal.h"

#if MUBL_USB && MUBL_USB_OTG_CORE

#define RX_WORDS 128U
#define TX_WORDS 16U
#define EP_CTRL 0U
#define WAIT 100000U

#ifdef MUBL_SIM
#include "hal/host/usb_otg_model.h"
#ifndef STM32_OTG_VBDEN
#define STM32_OTG_VBDEN 1
#endif
#else
#include "stm32.h"

static inline uint32_t rd(uint32_t off) { return REG32(STM32_OTG + off); }
static inline void wr(uint32_t off, uint32_t v) { REG32(STM32_OTG + off) = v; }

static bool hw_start(void)
{
    const bool ok = stm32_usb_clock(true);
#ifdef MUBL_USB_DM_PIN
    stm32_pin_af(MUBL_USB_DM_PIN, false, false);
    stm32_pin_af(MUBL_USB_DP_PIN, false, false);
#endif
    OTG_CLOCK_ENR |= OTG_CLOCK_BIT;
    OTG_RSTR |= OTG_CLOCK_BIT;
    OTG_RSTR &= ~OTG_CLOCK_BIT;
    (void)OTG_CLOCK_ENR;
    return ok;
}

static void hw_stop(void)
{
    OTG_RSTR |= OTG_CLOCK_BIT;
    OTG_RSTR &= ~OTG_CLOCK_BIT;
    OTG_CLOCK_ENR &= ~OTG_CLOCK_BIT;
    (void)stm32_usb_clock(false);
}
#endif

static void set(uint32_t off, uint32_t bits) { wr(off, rd(off) | bits); }
static void clear(uint32_t off, uint32_t bits) { wr(off, rd(off) & ~bits); }

/* Endpoint control: the stored fields as read, plus `bits`.  The command bits read as 0 (CNAK,
 * SNAK, SD0PID) or are kept from changing (EPENA, EPDIS). */
static void ctl(uint32_t off, uint32_t keep_clear, uint32_t bits)
{
    wr(off, (rd(off) & ~(EPCTL_EPENA | EPCTL_EPDIS | keep_clear)) | bits);
}

static bool wait(uint32_t off, uint32_t mask, uint32_t value)
{
    for (uint32_t i = 0; i < WAIT; i++) {
        if ((rd(off) & mask) == value) return true;
    }
    return false;
}

/* ---- State --------------------------------------------------------------------------------- */

enum { CTRL_IDLE, CTRL_DATA_IN, CTRL_DATA_OUT, CTRL_STATUS_IN, CTRL_STATUS_OUT };

MUBL_STATIC_ASSERT((MUBL_USB_TX_SIZE & (MUBL_USB_TX_SIZE - 1)) == 0 && MUBL_USB_TX_SIZE >= 64 &&
                   MUBL_USB_TX_SIZE <= 32768, "MUBL_USB_TX_SIZE must be a power of two, 64 to 32768");

static struct {
    uint8_t state;
    uint8_t in_zlp;
    uint8_t last_out;              /* size of the last OUT data packet on EP0 */
    const uint8_t* in_data;
    uint16_t in_left;
    uint16_t out_len;
    uint16_t out_got;
    uint8_t setup[8];
} ctrl;

static struct {
    uint8_t halted;                /* bit n: endpoint n halted */
    uint8_t tx_busy;               /* a packet is in the IN endpoint's FIFO */
    uint8_t tx_zlp;                /* the last packet was full-size */
    uint8_t rx_len, rx_pos;
    uint8_t rx_got;                /* bytes of the OUT packet being received */
    uint16_t tx_head, tx_tail;
    uint8_t rx[USB_DATA_SIZE];
    uint8_t tx[MUBL_USB_TX_SIZE];
} cdc;

/* ---- FIFOs ---------------------------------------------------------------------------------- */

static void fifo_write(uint8_t ep, const uint8_t* p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 4U) {
        uint32_t w = 0;
        for (uint16_t k = 0; k < 4U && i + k < n; k++) w |= (uint32_t)p[i + k] << (8U * k);
        wr(OTG_FIFO(ep), w);
    }
}

/* Read `n` bytes of the popped entry; `room` of them are kept in `out`. */
static void fifo_read(uint8_t* out, uint16_t n, uint16_t room)
{
    for (uint16_t i = 0; i < n; i += 4U) {
        const uint32_t w = rd(OTG_FIFO(0));
        for (uint16_t k = 0; k < 4U && i + k < n; k++) {
            if (i + k < room) out[i + k] = (uint8_t)(w >> (8U * k));
        }
    }
}

static void flush_tx(uint32_t fifo)
{
    (void)wait(OTG_GRSTCTL, GRSTCTL_AHBIDL, GRSTCTL_AHBIDL);
    wr(OTG_GRSTCTL, GRSTCTL_TXFFLSH | (fifo << GRSTCTL_TXFNUM_POS));
    (void)wait(OTG_GRSTCTL, GRSTCTL_TXFFLSH, 0);
}

/* Send one packet (or a zero-length one) on an IN endpoint. */
static void in_packet(uint8_t ep, const uint8_t* p, uint16_t n)
{
    wr(OTG_DIEPTSIZ(ep), (1U << EPTSIZ_PKTCNT_POS) | n);
    ctl(OTG_DIEPCTL(ep), EPCTL_STALL, EPCTL_EPENA | EPCTL_CNAK);
    fifo_write(ep, p, n);
}

/* Stop an IN endpoint's transfer and drop what is in its FIFO. */
static void in_abort(uint8_t ep)
{
    if (rd(OTG_DIEPCTL(ep)) & EPCTL_EPENA) {
        ctl(OTG_DIEPCTL(ep), 0, EPCTL_SNAK);
        (void)wait(OTG_DIEPINT(ep), DIEPINT_INEPNE, DIEPINT_INEPNE);
        ctl(OTG_DIEPCTL(ep), 0, EPCTL_SNAK | EPCTL_EPDIS);
        (void)wait(OTG_DIEPINT(ep), EPINT_EPDISD, EPINT_EPDISD);
    }
    wr(OTG_DIEPINT(ep), EPINT_XFRC | EPINT_EPDISD | DIEPINT_INEPNE);
    flush_tx(ep);
}

/* ---- Control transfers ------------------------------------------------------------------- */

/* EP0 OUT ready for a data or status stage (SETUP packets are taken whatever its state). */
static void ep0_out_arm(void)
{
    wr(OTG_DOEPTSIZ(0), DOEPTSIZ0_STUPCNT | (1U << EPTSIZ_PKTCNT_POS) | USB_EP0_SIZE);
    ctl(OTG_DOEPCTL(0), EPCTL_STALL, EPCTL_EPENA | EPCTL_CNAK);
}

static void ctrl_stall(void)
{
    ctrl.state = CTRL_IDLE;                      /* the core clears the stalls on the next SETUP */
    ctl(OTG_DIEPCTL(0), 0, EPCTL_STALL);
    ctl(OTG_DOEPCTL(0), 0, EPCTL_STALL);
}

void usbd_ack(void)
{
    ctrl.state = CTRL_STATUS_IN;
    in_packet(EP_CTRL, NULL, 0);
}

static void ctrl_in_packet(void)
{
    const uint16_t n = ctrl.in_left < USB_EP0_SIZE ? ctrl.in_left : USB_EP0_SIZE;
    const uint8_t* p = ctrl.in_data;
    ctrl.in_data += n;
    ctrl.in_left = (uint16_t)(ctrl.in_left - n);
    if (n < USB_EP0_SIZE) ctrl.in_zlp = 0;      /* a short packet ends the data stage */
    in_packet(EP_CTRL, p, n);
}

void usbd_reply(const uint8_t* data, uint16_t len)
{
    if (usb_req.wLength == 0U) {
        usbd_ack();
        return;
    }
    if (len > usb_req.wLength) len = usb_req.wLength;
    ctrl.in_data = data;
    ctrl.in_left = len;
    ctrl.in_zlp = len < usb_req.wLength && (len % USB_EP0_SIZE) == 0U;
    ctrl.state = CTRL_DATA_IN;
    ep0_out_arm();                               /* the host may end the data stage early */
    ctrl_in_packet();
}

bool usbd_receive(uint16_t len)
{
    if (len > sizeof usb_ctrl_buf) return false;
    ctrl.out_len = len;
    ctrl.out_got = 0;
    ctrl.state = CTRL_DATA_OUT;
    ep0_out_arm();
    return true;
}

void usbd_set_address(uint8_t address)
{
    /* This core takes the address at once; the status stage still goes out from address 0. */
    wr(OTG_DCFG, (rd(OTG_DCFG) & ~DCFG_DAD) | ((uint32_t)address << DCFG_DAD_POS));
    usbd_ack();
}

static void ctrl_setup(void)
{
    in_abort(EP_CTRL);                           /* a new SETUP abandons the transfer */
    ctrl.state = CTRL_IDLE;
    /* Room for the next SETUPs, unless EP0 OUT is armed for a data or status stage. */
    if (!(rd(OTG_DOEPCTL(0)) & EPCTL_EPENA)) {
        wr(OTG_DOEPTSIZ(0), DOEPTSIZ0_STUPCNT | (1U << EPTSIZ_PKTCNT_POS) | 24U);
    }
    usb_req.bmRequestType = ctrl.setup[0];
    usb_req.bRequest = ctrl.setup[1];
    usb_req.wValue = mubl_get16(&ctrl.setup[2]);
    usb_req.wIndex = mubl_get16(&ctrl.setup[4]);
    usb_req.wLength = mubl_get16(&ctrl.setup[6]);
    if (!usb_setup()) ctrl_stall();
}

static void ctrl_out_done(void)
{
    if (ctrl.state == CTRL_DATA_OUT) {
        if (ctrl.out_got < ctrl.out_len && ctrl.last_out == USB_EP0_SIZE) {
            ep0_out_arm();
        } else if (ctrl.out_got == ctrl.out_len && usb_setup_out()) {
            usbd_ack();
        } else {
            ctrl_stall();
        }
    } else if (ctrl.state == CTRL_DATA_IN || ctrl.state == CTRL_STATUS_OUT) {
        in_abort(EP_CTRL);                       /* the status stage ends the transfer */
        ctrl.state = CTRL_IDLE;
    }
}

static void ctrl_in_done(void)
{
    if (ctrl.state == CTRL_DATA_IN) {
        if (ctrl.in_left || ctrl.in_zlp) {
            ctrl_in_packet();
        } else {
            ctrl.state = CTRL_STATUS_OUT;
        }
    } else if (ctrl.state == CTRL_STATUS_IN) {
        ctrl.state = CTRL_IDLE;
    }
}

/* ---- Data endpoints ----------------------------------------------------------------------- */

static void out_arm(void)
{
    wr(OTG_DOEPTSIZ(USB_EP_OUT), (1U << EPTSIZ_PKTCNT_POS) | USB_DATA_SIZE);
    ctl(OTG_DOEPCTL(USB_EP_OUT), 0, EPCTL_EPENA | EPCTL_CNAK);
}

/* Disable an active OUT endpoint: under global OUT NAK, as the core requires. */
static void out_abort(uint8_t ep)
{
    if (rd(OTG_DOEPCTL(ep)) & EPCTL_EPENA) {
        set(OTG_DCTL, DCTL_SGONAK);
        (void)wait(OTG_GINTSTS, GINTSTS_GONAKEFF, GINTSTS_GONAKEFF);
        ctl(OTG_DOEPCTL(ep), 0, EPCTL_SNAK | EPCTL_EPDIS);
        (void)wait(OTG_DOEPINT(ep), EPINT_EPDISD, EPINT_EPDISD);
        set(OTG_DCTL, DCTL_CGONAK);
    }
    wr(OTG_DOEPINT(ep), EPINT_XFRC | EPINT_EPDISD);
}

static void cdc_tx_pump(void)
{
    uint8_t packet[USB_DATA_SIZE];
    if (!usb_configuration || cdc.tx_busy || (cdc.halted & (1U << USB_EP_IN))) return;
    const uint16_t used = (uint16_t)(cdc.tx_head - cdc.tx_tail);
    if (used == 0U && !cdc.tx_zlp) return;
    const uint16_t n = used < USB_DATA_SIZE ? used : USB_DATA_SIZE;
    for (uint16_t i = 0; i < n; i++) packet[i] = cdc.tx[(uint16_t)(cdc.tx_tail + i) & (MUBL_USB_TX_SIZE - 1U)];
    cdc.tx_tail = (uint16_t)(cdc.tx_tail + n);
    cdc.tx_zlp = n == USB_DATA_SIZE;
    cdc.tx_busy = 1;
    in_packet(USB_EP_IN, packet, n);
}

void usbd_configure(uint8_t value)
{
    memset(&cdc, 0, offsetof(__typeof__(cdc), rx));
    in_abort(USB_EP_IN);
    in_abort(USB_EP_NOTIFY);
    out_abort(USB_EP_OUT);
    if (!value) {
        wr(OTG_DIEPCTL(USB_EP_IN), 0);
        wr(OTG_DIEPCTL(USB_EP_NOTIFY), 0);
        wr(OTG_DOEPCTL(USB_EP_OUT), 0);
        return;
    }
    wr(OTG_DIEPCTL(USB_EP_IN), EPCTL_USBAEP | (EPTYP_BULK << EPCTL_EPTYP_POS) |
                                   ((uint32_t)USB_EP_IN << EPCTL_TXFNUM_POS) | USB_DATA_SIZE |
                                   EPCTL_SD0PID | EPCTL_SNAK);
    wr(OTG_DIEPCTL(USB_EP_NOTIFY), EPCTL_USBAEP | (EPTYP_INTERRUPT << EPCTL_EPTYP_POS) |
                                       ((uint32_t)USB_EP_NOTIFY << EPCTL_TXFNUM_POS) | USB_NOTIFY_SIZE |
                                       EPCTL_SD0PID | EPCTL_SNAK);
    wr(OTG_DOEPCTL(USB_EP_OUT), EPCTL_USBAEP | (EPTYP_BULK << EPCTL_EPTYP_POS) | USB_DATA_SIZE |
                                    EPCTL_SD0PID | EPCTL_SNAK);
    out_arm();
}

static uint8_t ep_number(uint16_t address)
{
    switch (address) {
    case USB_EP_IN_ADDR: return USB_EP_IN;
    case USB_EP_OUT_ADDR: return USB_EP_OUT;
    case USB_EP_NOTIFY_ADDR: return USB_EP_NOTIFY;
    default: return 0;
    }
}

int usbd_halted(uint16_t address)
{
    if (address == 0x00U || address == 0x80U) return 0;
    const uint8_t ep = ep_number(address);
    if (ep == 0U || !usb_configuration) return -1;
    return (cdc.halted >> ep) & 1;
}

/* Clearing a halt resets the data toggle (USB 2.0 9.4.5).  A packet waiting in the IN FIFO stays
 * there and goes once the halt is cleared; received data not yet read keeps OUT disarmed. */
bool usbd_halt(uint16_t address, bool halt)
{
    if (address == 0x00U || address == 0x80U) return !halt;
    const uint8_t ep = ep_number(address);
    if (ep == 0U || !usb_configuration) return false;
    const uint32_t reg = ep == USB_EP_OUT ? OTG_DOEPCTL(ep) : OTG_DIEPCTL(ep);
    if (halt) {
        cdc.halted |= (uint8_t)(1U << ep);
        ctl(reg, 0, EPCTL_STALL);
        return true;
    }
    cdc.halted &= (uint8_t)~(1U << ep);
    ctl(reg, EPCTL_STALL, EPCTL_SD0PID);
    if (ep == USB_EP_OUT && cdc.rx_len == 0U && !(rd(reg) & EPCTL_EPENA)) out_arm();
    return true;
}

/* ---- Bus ----------------------------------------------------------------------------------- */

static void bus_reset(void)
{
    flush_tx(0x10U);
    for (uint8_t ep = 0; ep < 4U; ep++) {
        wr(OTG_DIEPINT(ep), 0xFFFFU);
        wr(OTG_DOEPINT(ep), 0xFFFFU);
        ctl(OTG_DIEPCTL(ep), EPCTL_STALL, 0);
        ctl(OTG_DOEPCTL(ep), EPCTL_STALL, EPCTL_SNAK);
    }
    wr(OTG_DCFG, rd(OTG_DCFG) & ~DCFG_DAD);
    wr(OTG_DOEPTSIZ(0), DOEPTSIZ0_STUPCNT | (1U << EPTSIZ_PKTCNT_POS) | 24U);
    memset(&ctrl, 0, sizeof ctrl);
    usb_cdc_reset();
    usbd_configure(0);
}

static bool core_init(void)
{
    set(OTG_GUSBCFG, GUSBCFG_PHYSEL);            /* embedded full-speed PHY */
    if (!wait(OTG_GRSTCTL, GRSTCTL_AHBIDL, GRSTCTL_AHBIDL)) return false;
    wr(OTG_GRSTCTL, GRSTCTL_CSRST);
    if (!wait(OTG_GRSTCTL, GRSTCTL_CSRST, 0)) return false;
    hal_delay_us(3);
#if STM32_OTG_VBDEN
    /* No VBUS sensing: the B-session is forced valid. */
    wr(OTG_GCCFG, GCCFG_PWRDWN);
    set(OTG_GOTGCTL, GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL);
#else
    wr(OTG_GCCFG, GCCFG_PWRDWN | GCCFG_NOVBUSSENS);
#endif
    wr(OTG_GUSBCFG, (rd(OTG_GUSBCFG) & ~(GUSBCFG_FHMOD | GUSBCFG_TRDT)) | GUSBCFG_FDMOD |
                        (6U << GUSBCFG_TRDT_POS));      /* turnaround for AHB >= 32 MHz */
    hal_delay_us(25000);                         /* forcing the mode takes 25 ms */
    for (uint32_t ms = 0; ms < 50U && (rd(OTG_GINTSTS) & GINTSTS_CMOD); ms++) hal_delay_us(1000);
    wr(OTG_DCTL, DCTL_SDIS);                     /* detached while setting up */
    wr(OTG_PCGCCTL, 0);
    wr(OTG_DCFG, DCFG_DSPD_FS);
    wr(OTG_GRXFSIZ, RX_WORDS);
    wr(OTG_DIEPTXF0, (TX_WORDS << 16) | RX_WORDS);
    for (uint32_t n = 1; n < 4U; n++) wr(OTG_DIEPTXF(n), (TX_WORDS << 16) | (RX_WORDS + TX_WORDS * n));
    flush_tx(0x10U);
    wr(OTG_GRSTCTL, GRSTCTL_RXFFLSH);
    (void)wait(OTG_GRSTCTL, GRSTCTL_RXFFLSH, 0);
    wr(OTG_DIEPMSK, 0);
    wr(OTG_DOEPMSK, 0);
    wr(OTG_DAINTMSK, 0);
    wr(OTG_GINTMSK, 0);
    wr(OTG_GAHBCFG, 0);                          /* global interrupt off: polled */
    wr(OTG_GINTSTS, 0xBFFFFFFFU);
    return true;
}

static uint8_t running;

void hal_usb_start(void)
{
    memset(&ctrl, 0, sizeof ctrl);
    memset(&cdc, 0, offsetof(__typeof__(cdc), rx));
    usb_cdc_reset();
    running = hw_start() && core_init();         /* no 48 MHz clock or no core: stay off the bus */
    if (!running) {
        hw_stop();
        return;
    }
    bus_reset();
    clear(OTG_DCTL, DCTL_SDIS);                  /* attach: D+ pull-up on */
}

/* Pop the receive FIFO's status entries, reading the data of each. */
static void rx_pop(void)
{
    for (uint8_t i = 0; i < 16U && (rd(OTG_GINTSTS) & GINTSTS_RXFLVL); i++) {
        const uint32_t st = rd(OTG_GRXSTSP);
        const uint8_t ep = (uint8_t)(st & GRXSTS_EPNUM);
        const uint16_t n = (uint16_t)((st & GRXSTS_BCNT) >> GRXSTS_BCNT_POS);
        const uint32_t kind = (st & GRXSTS_PKTSTS) >> GRXSTS_PKTSTS_POS;
        if (kind == PKTSTS_SETUP_DATA) {
            fifo_read(ctrl.setup, n, sizeof ctrl.setup);
        } else if (kind == PKTSTS_OUT_DATA && ep == EP_CTRL) {
            const uint16_t room = ctrl.state == CTRL_DATA_OUT ? (uint16_t)(ctrl.out_len - ctrl.out_got) : 0U;
            fifo_read(usb_ctrl_buf + ctrl.out_got, n, room);
            ctrl.out_got = (uint16_t)(ctrl.out_got + (n < room ? n : room));
            ctrl.last_out = (uint8_t)(n > room ? USB_EP0_SIZE + 1U : n);   /* too long: stalls */
        } else if (kind == PKTSTS_OUT_DATA && ep == USB_EP_OUT) {
            fifo_read(cdc.rx, n, sizeof cdc.rx);
            cdc.rx_got = (uint8_t)(n < sizeof cdc.rx ? n : sizeof cdc.rx);
        } else {
            fifo_read(NULL, n, 0);
        }
    }
}

uint8_t hal_usb_poll(void)
{
    uint8_t events = 0;
    if (!running) return 0;
    const uint32_t g = rd(OTG_GINTSTS);
    if (g & GINTSTS_USBRST) {
        wr(OTG_GINTSTS, GINTSTS_USBRST);
        bus_reset();
        events |= HAL_USB_RESET;
    }
    if (g & GINTSTS_ENUMDNE) {
        wr(OTG_GINTSTS, GINTSTS_ENUMDNE);
        ctl(OTG_DIEPCTL(0), EPCTL_MPSIZ, 0);     /* EP0: 64 bytes */
        set(OTG_DCTL, DCTL_CGINAK);
    }
    if (g & GINTSTS_SOF) {
        wr(OTG_GINTSTS, GINTSTS_SOF);
        events |= HAL_USB_SOF;
    }
    rx_pop();

    uint32_t v = rd(OTG_DOEPINT(0));
    if (v & EPINT_XFRC) {                        /* a data or status stage packet arrived */
        wr(OTG_DOEPINT(0), EPINT_XFRC);
        ctrl_out_done();
    }
    if (v & DOEPINT_STUP) {
        wr(OTG_DOEPINT(0), DOEPINT_STUP);
        ctrl_setup();
    }
    if (rd(OTG_DIEPINT(0)) & EPINT_XFRC) {
        wr(OTG_DIEPINT(0), EPINT_XFRC);
        ctrl_in_done();
    }
    if (rd(OTG_DIEPINT(USB_EP_IN)) & EPINT_XFRC) {
        wr(OTG_DIEPINT(USB_EP_IN), EPINT_XFRC);
        cdc.tx_busy = 0;
    }
    if (rd(OTG_DOEPINT(USB_EP_OUT)) & EPINT_XFRC) {
        wr(OTG_DOEPINT(USB_EP_OUT), EPINT_XFRC);
        cdc.rx_len = cdc.rx_got;
        cdc.rx_pos = 0;
        if (cdc.rx_len == 0U && !(cdc.halted & (1U << USB_EP_OUT))) out_arm();   /* a ZLP */
    }
    cdc_tx_pump();
    if (usb_configuration) events |= HAL_USB_CONFIGURED;
    return events;
}

int hal_usb_getc(void)
{
    if (cdc.rx_pos >= cdc.rx_len) return -1;
    const uint8_t c = cdc.rx[cdc.rx_pos++];
    if (cdc.rx_pos == cdc.rx_len) {
        cdc.rx_len = cdc.rx_pos = 0;
        if (usb_configuration && !(cdc.halted & (1U << USB_EP_OUT))) out_arm();
    }
    return c;
}

uint16_t usbd_tx_space(void)
{
    return usb_configuration ? (uint16_t)(MUBL_USB_TX_SIZE - (uint16_t)(cdc.tx_head - cdc.tx_tail)) : 0U;
}

bool usbd_tx_empty(void)
{
    return cdc.tx_head == cdc.tx_tail && !cdc.tx_busy && !cdc.tx_zlp;
}

bool hal_usb_write(const uint8_t* data, uint16_t n)
{
    if (n > usbd_tx_space()) return false;
    for (uint16_t i = 0; i < n; i++) cdc.tx[(uint16_t)(cdc.tx_head + i) & (MUBL_USB_TX_SIZE - 1U)] = data[i];
    cdc.tx_head = (uint16_t)(cdc.tx_head + n);
    cdc_tx_pump();
    return true;
}

void hal_usb_stop(void)
{
    for (uint32_t i = 0; i < MUBL_USB_FLUSH_MS * 10U && usb_configuration && !usbd_tx_empty(); i++) {
        (void)hal_usb_poll();
        hal_delay_us(100);
    }
    if (!running) return;
    running = 0;
    set(OTG_DCTL, DCTL_SDIS);                    /* detach */
    usb_cdc_reset();
    hal_delay_us(MUBL_USB_DETACH_MS * 1000U);
    hw_stop();
}

#endif
