/*
 * The full-speed USB device peripheral with packet memory (STM32F0, F3): the device layer of the
 * CDC-ACM port (usb_cdc.h).  Ported from µCAN's usb/usb_cdc.c, which runs on the F042 in the field;
 * its rules hold here:
 *
 *  - Single context.  hal_usb_poll() from the main loop is the only code that touches the
 *    peripheral; the USB interrupt stays disabled.
 *  - EPnR has rw, toggle and rc_w0 bits, so every write is built explicitly and never clears a
 *    CTR flag it did not mean to clear.
 *  - An endpoint is held at NAK, never DISABLED, while the driver decides what comes next: a
 *    disabled endpoint gives the host no handshake, which hosts count as a transaction error.
 *  - Bulk IN is fed from a byte ring; a zero-length packet follows a full packet that empties it.
 *  - Bulk OUT is flow controlled: the endpoint is re-armed only once the previous packet has been
 *    read, so the host is held off (NAK) while the bootloader is busy.
 *
 * Packet memory: F0 and F303xD/E address it as 16-bit words at consecutive 16-bit addresses;
 * F303xB/C as 16-bit words at 32-bit addresses (STM32_USB_PMA_STRIDE 2).  With CAN clocked, the
 * last 256 bytes of a 1 KiB packet memory belong to CAN: the layout here stays below 0x150.
 */
#include "usb_cdc.h"
#include "usb_fs_regs.h"

#include "hal/hal.h"

#if MUBL_USB && !MUBL_USB_OTG_CORE

/* Packet memory layout, byte offsets */
#define PMA_BTABLE 0x000U          /* 8 bytes per endpoint */
#define PMA_EP0_TX 0x040U
#define PMA_EP0_RX 0x080U
#define PMA_IN 0x0C0U
#define PMA_OUT 0x100U
#define PMA_NOTIFY 0x140U

#define BT_ADDR_TX 0U              /* buffer descriptor fields, in 16-bit words */
#define BT_COUNT_TX 1U
#define BT_ADDR_RX 2U
#define BT_COUNT_RX 3U
#define BT_COUNT_MASK 0x03FFU

#define EP_CTRL 0U

/* ---- Hardware access (the host simulation substitutes a model of the peripheral) ---------- */

#ifdef MUBL_SIM
#include "hal/host/usb_fs_model.h"
#define PMA_STEP 1U
#else
#include "stm32.h"

#define PMA_STEP STM32_USB_PMA_STRIDE
#define USB_REG(off) (*(volatile uint16_t*)(uintptr_t)(STM32_USB + (off)))

static inline uint32_t hw_irq_save(void)
{
    uint32_t primask;
    __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory");
    return primask;
}

static inline void hw_irq_restore(uint32_t primask)
{
    __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
}

static inline uint16_t hw_epr_get(uint8_t ep) { return USB_REG(USB_EPR(ep)); }
static inline void hw_epr_set(uint8_t ep, uint16_t v) { USB_REG(USB_EPR(ep)) = v; }
static inline uint16_t hw_istr_get(void) { return USB_REG(USB_ISTR); }
static inline void hw_istr_clear(uint16_t bits) { USB_REG(USB_ISTR) = (uint16_t)~bits; }   /* rc_w0 */
static inline void hw_set_address(uint8_t a) { USB_REG(USB_DADDR) = (uint16_t)(USB_DADDR_EF | a); }

static inline volatile uint16_t* hw_pma(uint16_t offset)
{
    return (volatile uint16_t*)(uintptr_t)(STM32_USB_PMA + (uint32_t)offset * STM32_USB_PMA_STRIDE);
}

/* Attach and detach: the internal pull-up (F0), a switched external one (MUBL_USB_PULLUP_PIN), or,
 * with a fixed external pull-up, D+ driven low while detached (the host sees SE0: a disconnect). */
#if !STM32_USB_DPPU && !defined(MUBL_USB_PULLUP_PIN)
#define FIXED_PULLUP 1
#else
#define FIXED_PULLUP 0
#endif

static bool hw_start(void)
{
    const bool ok = stm32_usb_clock(true);
#if FIXED_PULLUP
    /* The application may have left the host thinking it is still attached: detach first, so the
     * host enumerates the bootloader afresh. */
    stm32_pin_output(MUBL_USB_DP_PIN, false);
    hal_delay_us(MUBL_USB_DETACH_MS * 1000U);
#endif
#ifdef MUBL_USB_DM_PIN
    stm32_pin_af(MUBL_USB_DM_PIN, false, false);
#if !FIXED_PULLUP
    stm32_pin_af(MUBL_USB_DP_PIN, false, false);
#endif
#endif
    USB_CLOCK_ENR |= USB_CLOCK_BIT;
    USB_REG(USB_CNTR) = USB_CNTR_FRES;           /* transceiver on (PDWN 0), core held in reset */
    hal_delay_us(2);                             /* t_STARTUP: 1 us */
    USB_REG(USB_CNTR) = 0;                       /* every interrupt masked: ISTR is polled */
    USB_REG(USB_ISTR) = 0;
    USB_REG(USB_BTABLE) = PMA_BTABLE;
    return ok;
}


static void hw_connect(void)
{
#ifdef MUBL_USB_PULLUP_PIN
    stm32_pin_output(MUBL_USB_PULLUP_PIN, MUBL_USB_PULLUP_LEVEL);
#endif
#if STM32_USB_DPPU
    USB_REG(USB_BCDR) |= USB_BCDR_DPPU;
#endif
#if FIXED_PULLUP
    stm32_pin_af(MUBL_USB_DP_PIN, false, false);
#endif
}

static void hw_disconnect(void)
{
#if STM32_USB_DPPU
    USB_REG(USB_BCDR) &= (uint16_t)~USB_BCDR_DPPU;
#endif
#ifdef MUBL_USB_PULLUP_PIN
    stm32_pin_write(MUBL_USB_PULLUP_PIN, !MUBL_USB_PULLUP_LEVEL);
#endif
#if FIXED_PULLUP
    stm32_pin_output(MUBL_USB_DP_PIN, false);
#endif
}

static void hw_stop(void)
{
    USB_RSTR |= USB_CLOCK_BIT;
    USB_RSTR &= ~USB_CLOCK_BIT;
    USB_CLOCK_ENR &= ~USB_CLOCK_BIT;
    (void)stm32_usb_clock(false);
}
#endif

/* ---- State --------------------------------------------------------------------------------- */

enum {
    CTRL_IDLE,
    CTRL_DATA_IN,       /* sending the data stage */
    CTRL_DATA_OUT,      /* receiving the data stage */
    CTRL_STATUS_IN,     /* zero-length status packet armed */
    CTRL_STATUS_OUT     /* waiting for the host's zero-length status packet */
};

MUBL_STATIC_ASSERT((MUBL_USB_TX_SIZE & (MUBL_USB_TX_SIZE - 1)) == 0 && MUBL_USB_TX_SIZE >= 64 &&
                   MUBL_USB_TX_SIZE <= 32768, "MUBL_USB_TX_SIZE must be a power of two, 64 to 32768");

static struct {
    uint8_t state;
    uint8_t set_address;           /* apply `address` once the status stage completes */
    uint8_t address;
    uint8_t in_zlp;                /* the data stage still needs a terminating short packet */
    const uint8_t* in_data;
    uint16_t in_left;
    uint16_t out_len;
    uint16_t out_got;
} ctrl;

static struct {
    uint8_t halted;                /* bit n: endpoint n halted */
    uint8_t tx_busy;               /* a packet is armed on the IN endpoint */
    uint8_t tx_zlp;                /* the last packet was full-size */
    uint8_t rx_len, rx_pos;
    uint16_t tx_head, tx_tail;     /* free-running; used = head - tail */
    uint8_t rx[USB_DATA_SIZE];
    uint8_t tx[MUBL_USB_TX_SIZE];
} cdc;

/* ---- Endpoints and packet memory -------------------------------------------------------- */

static volatile uint16_t* btable(uint8_t ep, uint8_t field)
{
    return hw_pma((uint16_t)(PMA_BTABLE + 8U * ep + 2U * field));
}

/* COUNTn_RX block encoding for a receive buffer of `size` bytes */
static uint16_t rx_count_field(uint16_t size)
{
    if (size > 62U) return (uint16_t)(0x8000U | (((size / 32U) - 1U) << 10));
    return (uint16_t)((size / 2U) << 10);
}

static uint16_t rx_count(uint8_t ep)
{
    return (uint16_t)(*btable(ep, BT_COUNT_RX) & BT_COUNT_MASK);
}

static void pma_write(uint16_t offset, const uint8_t* src, uint16_t len)
{
    volatile uint16_t* dst = hw_pma(offset);
    for (uint16_t i = 0; i < len; i += 2U, dst += PMA_STEP) {
        uint16_t w = src[i];
        if (i + 1U < len) w |= (uint16_t)(src[i + 1U] << 8);
        *dst = w;
    }
}

static void pma_read(uint16_t offset, uint8_t* dst, uint16_t len)
{
    volatile uint16_t* src = hw_pma(offset);
    for (uint16_t i = 0; i < len; i += 2U, src += PMA_STEP) {
        const uint16_t w = *src;
        dst[i] = (uint8_t)w;
        if (i + 1U < len) dst[i + 1U] = (uint8_t)(w >> 8);
    }
}

static void ep_set_status(uint8_t ep, uint16_t rx, uint16_t tx)
{
    const uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ rx) & EPR_STAT_RX) | ((r ^ tx) & EPR_STAT_TX)));
}

static void ep_set_rx_status(uint8_t ep, uint16_t rx)
{
    const uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ rx) & EPR_STAT_RX)));
}

static void ep_set_tx_status(uint8_t ep, uint16_t tx)
{
    const uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ tx) & EPR_STAT_TX)));
}

/* Hold an endpoint at NAK.  A transaction can finish between the status read and the toggle
 * write (VALID becomes NAK), turning that write into STALL: re-read until the field settles on
 * NAK, with interrupts off so the transient STALL stays short. */
static void ep_nak(uint8_t ep, uint16_t mask, uint16_t nak)
{
    const uint32_t irq = hw_irq_save();
    uint16_t r;
    while (((r = hw_epr_get(ep)) & mask) != nak) {
        hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ nak) & mask)));
    }
    hw_irq_restore(irq);
}

static void ep_clear_ctr_rx(uint8_t ep)
{
    hw_epr_set(ep, (uint16_t)((hw_epr_get(ep) & EPR_RW) | EPR_CTR_TX));
}

static void ep_clear_ctr_tx(uint8_t ep)
{
    hw_epr_set(ep, (uint16_t)((hw_epr_get(ep) & EPR_RW) | EPR_CTR_RX));
}

/* Type and address set, both CTR flags cleared, both toggles back to DATA0, both statuses set. */
static void ep_configure(uint8_t ep, uint16_t type, uint16_t rx, uint16_t tx)
{
    const uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)(type | ep | (r & (EPR_DTOG_RX | EPR_DTOG_TX)) | ((r ^ rx) & EPR_STAT_RX) |
                              ((r ^ tx) & EPR_STAT_TX)));
}

/* ---- CDC data path ------------------------------------------------------------------------ */

static void cdc_tx_pump(void)
{
    uint8_t packet[USB_DATA_SIZE];
    if (!usb_configuration || cdc.tx_busy || (cdc.halted & (1U << USB_EP_IN))) return;
    const uint16_t used = (uint16_t)(cdc.tx_head - cdc.tx_tail);
    if (used == 0U && !cdc.tx_zlp) return;
    const uint16_t n = used < USB_DATA_SIZE ? used : USB_DATA_SIZE;
    for (uint16_t i = 0; i < n; i++) packet[i] = cdc.tx[(uint16_t)(cdc.tx_tail + i) & (MUBL_USB_TX_SIZE - 1U)];
    cdc.tx_tail = (uint16_t)(cdc.tx_tail + n);
    cdc.tx_zlp = n == USB_DATA_SIZE;         /* a full packet that empties the ring: a ZLP next */
    cdc.tx_busy = 1;
    pma_write(PMA_IN, packet, n);
    *btable(USB_EP_IN, BT_COUNT_TX) = n;
    ep_set_tx_status(USB_EP_IN, TX_VALID);
}

static void cdc_out_received(void)
{
    uint16_t n = rx_count(USB_EP_OUT);
    if (n > USB_DATA_SIZE) n = USB_DATA_SIZE;
    pma_read(PMA_OUT, cdc.rx, n);
    ep_clear_ctr_rx(USB_EP_OUT);
    cdc.rx_len = (uint8_t)n;
    cdc.rx_pos = 0;
    /* The hardware left the endpoint NAKing: re-arm only once the data has been read. */
    if (n == 0U && !(cdc.halted & (1U << USB_EP_OUT))) ep_set_rx_status(USB_EP_OUT, RX_VALID);
}

void usbd_configure(uint8_t value)
{
    memset(&cdc, 0, offsetof(__typeof__(cdc), rx));
    if (value) {
        ep_configure(USB_EP_IN, EPR_TYPE_BULK, RX_DIS, TX_NAK);
        ep_configure(USB_EP_OUT, EPR_TYPE_BULK, RX_VALID, TX_DIS);
        ep_configure(USB_EP_NOTIFY, EPR_TYPE_INTERRUPT, RX_DIS, TX_NAK);
    } else {
        ep_configure(USB_EP_IN, EPR_TYPE_BULK, RX_DIS, TX_DIS);
        ep_configure(USB_EP_OUT, EPR_TYPE_BULK, RX_DIS, TX_DIS);
        ep_configure(USB_EP_NOTIFY, EPR_TYPE_INTERRUPT, RX_DIS, TX_DIS);
    }
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

/* Clearing a halt always resets the data toggle (USB 2.0 9.4.5). */
bool usbd_halt(uint16_t address, bool halt)
{
    if (address == 0x00U || address == 0x80U) return !halt;   /* EP0 cannot be halted */
    const uint8_t ep = ep_number(address);
    if (ep == 0U || !usb_configuration) return false;
    if (ep == USB_EP_IN) {
        ep_nak(USB_EP_IN, EPR_STAT_TX, TX_NAK);
        /* EP0 has priority over the bulk completion: a packet may have reached the host with its
         * CTR_TX still set. */
        if (hw_epr_get(USB_EP_IN) & EPR_CTR_TX) {
            ep_clear_ctr_tx(USB_EP_IN);
            cdc.tx_busy = 0;
        }
    } else if (ep == USB_EP_OUT) {
        ep_nak(USB_EP_OUT, EPR_STAT_RX, RX_NAK);
        /* Keep an OUT packet acknowledged before this request, seen or not. */
        cdc.halted |= (uint8_t)(1U << USB_EP_OUT);
        if (hw_epr_get(USB_EP_OUT) & EPR_CTR_RX) cdc_out_received();
    }
    if (halt) {
        cdc.halted |= (uint8_t)(1U << ep);
        if (ep == USB_EP_OUT) {
            ep_set_rx_status(ep, RX_STALL);
        } else {
            ep_set_tx_status(ep, TX_STALL);
        }
        return true;
    }
    cdc.halted &= (uint8_t)~(1U << ep);
    if (ep == USB_EP_IN) {
        /* tx_tail moved on when the packet went to packet memory: keep an unacknowledged one armed. */
        ep_configure(USB_EP_IN, EPR_TYPE_BULK, RX_DIS, cdc.tx_busy ? TX_VALID : TX_NAK);
    } else if (ep == USB_EP_OUT) {
        ep_configure(USB_EP_OUT, EPR_TYPE_BULK, cdc.rx_len ? RX_NAK : RX_VALID, TX_DIS);
    } else {
        ep_configure(USB_EP_NOTIFY, EPR_TYPE_INTERRUPT, RX_DIS, TX_NAK);
    }
    return true;
}

/* ---- Control transfers ------------------------------------------------------------------- */

/* EP0's statuses for the next stage.  A SETUP (or OUT) completing between the status read and
 * write would leave this stage's packet or STALL armed for the next request: hold both
 * directions at NAK then, and let the next poll handle it. */
static void ctrl_set_status(uint16_t rx, uint16_t tx)
{
    ep_set_status(EP_CTRL, rx, tx);
    if (hw_epr_get(EP_CTRL) & EPR_CTR_RX) ep_nak(EP_CTRL, EPR_STAT_RX | EPR_STAT_TX, RX_NAK | TX_NAK);
}

static void ctrl_stall(void)
{
    ctrl.state = CTRL_IDLE;                      /* the next SETUP is still accepted */
    ctrl_set_status(RX_STALL, TX_STALL);
}

void usbd_ack(void)
{
    ctrl.state = CTRL_STATUS_IN;
    *btable(EP_CTRL, BT_COUNT_TX) = 0;
    ctrl_set_status(RX_VALID, TX_VALID);
}

static void ctrl_in_packet(void)
{
    const uint16_t n = ctrl.in_left < USB_EP0_SIZE ? ctrl.in_left : USB_EP0_SIZE;
    pma_write(PMA_EP0_TX, ctrl.in_data, n);
    *btable(EP_CTRL, BT_COUNT_TX) = n;
    ctrl.in_data += n;
    ctrl.in_left = (uint16_t)(ctrl.in_left - n);
    if (n < USB_EP0_SIZE) ctrl.in_zlp = 0;      /* a short packet ends the data stage */
    ctrl_set_status(RX_VALID, TX_VALID);         /* the host may move to the status stage early */
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
    ctrl_in_packet();
}

bool usbd_receive(uint16_t len)
{
    if (len > sizeof usb_ctrl_buf) return false;
    ctrl.out_len = len;
    ctrl.out_got = 0;
    ctrl.state = CTRL_DATA_OUT;
    ctrl_set_status(RX_VALID, TX_NAK);
    return true;
}

void usbd_set_address(uint8_t address)
{
    ctrl.address = address;                      /* takes effect after the status stage */
    ctrl.set_address = 1;
    usbd_ack();
}

static void ctrl_setup_received(void)
{
    uint8_t raw[8];
    const uint16_t n = rx_count(EP_CTRL);
    pma_read(PMA_EP0_RX, raw, sizeof raw);       /* before clearing CTR_RX: a new SETUP may land */
    ep_clear_ctr_rx(EP_CTRL);
    ctrl.state = CTRL_IDLE;                      /* a transfer in progress is abandoned */
    ctrl.set_address = 0;
    if (n != sizeof raw) {
        ctrl_stall();
        return;
    }
    usb_req.bmRequestType = raw[0];
    usb_req.bRequest = raw[1];
    usb_req.wValue = mubl_get16(&raw[2]);
    usb_req.wIndex = mubl_get16(&raw[4]);
    usb_req.wLength = mubl_get16(&raw[6]);
    if (!usb_setup()) ctrl_stall();
}

static void ctrl_out_received(void)
{
    const uint16_t n = rx_count(EP_CTRL);
    if (ctrl.state != CTRL_DATA_OUT) {
        /* The status stage of an IN transfer (perhaps ending it early) or a stray packet: either
         * way the transfer is over. */
        ep_clear_ctr_rx(EP_CTRL);
        ctrl.state = CTRL_IDLE;
        ctrl_set_status(RX_VALID, TX_NAK);
        return;
    }
    if (n > (uint16_t)(ctrl.out_len - ctrl.out_got)) {
        ep_clear_ctr_rx(EP_CTRL);
        ctrl_stall();
        return;
    }
    pma_read(PMA_EP0_RX, usb_ctrl_buf + ctrl.out_got, n);
    ep_clear_ctr_rx(EP_CTRL);
    ctrl.out_got = (uint16_t)(ctrl.out_got + n);
    if (ctrl.out_got < ctrl.out_len && n == USB_EP0_SIZE) {
        ctrl_set_status(RX_VALID, TX_NAK);       /* more of the data stage to come */
    } else if (ctrl.out_got == ctrl.out_len && usb_setup_out()) {
        usbd_ack();
    } else {
        ctrl_stall();
    }
}

static void ctrl_in_sent(bool pending_rx)
{
    ep_clear_ctr_tx(EP_CTRL);
    switch (ctrl.state) {
    case CTRL_DATA_IN:
        /* A new SETUP aborts this data stage and a status OUT ends it: never arm another old
         * packet before handling either. */
        if (pending_rx) break;
        if (ctrl.in_left || ctrl.in_zlp) {
            ctrl_in_packet();
            return;
        }
        ctrl.state = CTRL_STATUS_OUT;
        break;
    case CTRL_STATUS_IN:
        if (ctrl.set_address) {
            hw_set_address(ctrl.address);
            ctrl.set_address = 0;
        }
        ctrl.state = CTRL_IDLE;
        break;
    default:
        break;
    }
    if (!pending_rx) ctrl_set_status(RX_VALID, TX_NAK);
}

/* ---- Bus ----------------------------------------------------------------------------------- */

static void bus_reset(void)
{
    static const uint16_t bt[] = {
        PMA_EP0_TX, 0, PMA_EP0_RX, 0,            /* EP0's RX count is set below */
        PMA_IN, 0, 0, 0,
        0, 0, PMA_OUT, 0,
        PMA_NOTIFY, 0, 0, 0,
    };
    for (uint8_t i = 0; i < sizeof bt / sizeof bt[0]; i++) *btable(i / 4U, i % 4U) = bt[i];
    *btable(EP_CTRL, BT_COUNT_RX) = rx_count_field(USB_EP0_SIZE);
    *btable(USB_EP_OUT, BT_COUNT_RX) = rx_count_field(USB_DATA_SIZE);
    ctrl.state = CTRL_IDLE;
    ctrl.set_address = 0;
    ep_configure(EP_CTRL, EPR_TYPE_CONTROL, RX_VALID, TX_NAK);
    usb_cdc_reset();
    usbd_configure(0);
    hw_set_address(0);
}

static uint8_t running;

void hal_usb_start(void)
{
    memset(&ctrl, 0, sizeof ctrl);
    memset(&cdc, 0, offsetof(__typeof__(cdc), rx));
    usb_cdc_reset();
    running = hw_start();                        /* no 48 MHz clock: stay off the bus */
    if (!running) {
        hw_stop();
        return;
    }
    bus_reset();
    hw_connect();
}

uint8_t hal_usb_poll(void)
{
    uint8_t events = 0;
    if (!running) return 0;
    const uint16_t istr = hw_istr_get();
    if (istr & ISTR_RESET) {
        hw_istr_clear(ISTR_RESET);
        bus_reset();
        events |= HAL_USB_RESET;
    }
    if (istr & ISTR_SOF) {
        hw_istr_clear(ISTR_SOF);
        events |= HAL_USB_SOF;
    }
    /* Each pass clears at least one CTR flag; the bound keeps the main loop moving when the host
     * is busy (the rest is handled next time). */
    for (uint8_t pass = 0; pass < 8U; pass++) {
        const uint16_t pending = hw_istr_get();
        if (!(pending & ISTR_CTR)) break;
        const uint8_t ep = (uint8_t)(pending & ISTR_EP_ID);
        uint16_t epr = hw_epr_get(ep);
        switch (ep) {
        case EP_CTRL:
            /* Both directions at NAK before clearing CTR or preparing a reply, then act on the
             * refreshed flags.  A SETUP is still accepted (and supersedes this transfer). */
            ep_nak(EP_CTRL, EPR_STAT_RX | EPR_STAT_TX, RX_NAK | TX_NAK);
            epr = hw_epr_get(EP_CTRL);
            if (epr & EPR_CTR_TX) ctrl_in_sent((epr & EPR_CTR_RX) != 0U);   /* SET_ADDRESS still applies */
            if (epr & EPR_CTR_RX) {
                if (epr & EPR_SETUP) {
                    ctrl_setup_received();
                } else {
                    ctrl_out_received();
                }
            }
            break;
        case USB_EP_IN:
            if (epr & EPR_CTR_TX) {
                ep_clear_ctr_tx(USB_EP_IN);
                cdc.tx_busy = 0;
            }
            if (epr & EPR_CTR_RX) ep_clear_ctr_rx(USB_EP_IN);
            break;
        case USB_EP_OUT:
            if (epr & EPR_CTR_RX) cdc_out_received();
            if (epr & EPR_CTR_TX) ep_clear_ctr_tx(USB_EP_OUT);
            break;
        default:
            if (epr & EPR_CTR_RX) ep_clear_ctr_rx(ep);
            if (epr & EPR_CTR_TX) ep_clear_ctr_tx(ep);
            break;
        }
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
        if (usb_configuration && !(cdc.halted & (1U << USB_EP_OUT))) ep_set_rx_status(USB_EP_OUT, RX_VALID);
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
    /* Let the host collect the last reply, then leave the bus long enough for it to notice. */
    for (uint32_t i = 0; i < MUBL_USB_FLUSH_MS * 10U && usb_configuration && !usbd_tx_empty(); i++) {
        (void)hal_usb_poll();
        hal_delay_us(100);
    }
    if (!running) return;
    running = 0;
    hw_disconnect();
    usb_cdc_reset();
    hal_delay_us(MUBL_USB_DETACH_MS * 1000U);
    hw_stop();
}

#endif
