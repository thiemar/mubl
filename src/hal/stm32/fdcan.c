/*
 * FDCAN (H7 and G4): one extended filter, receive FIFO 0 and transmit FIFO (sent in order),
 * polled.  Classic CAN unless MUBL_CAN_FD, which sends CAN FD frames, with bit-rate switching if
 * MUBL_CAN_FD_DATA_BITRATE is set.
 *
 * On H7 the driver lays out message RAM (16-element FIFOs); on G4 message RAM has a fixed 212-word
 * layout per instance (RM0440 §44.3.3: 3-element FIFOs, always 64-byte capable).
 * The kernel clock is HSE (FDCANSEL's reset value), so CAN timing needs MUBL_HSE_HZ.
 */
#include "can_timing.h"
#include "hal/hal.h"
#include "stm32.h"

#if MUBL_DRONECAN && (MUBL_FAMILY == MUBL_FAMILY_H7 || MUBL_FAMILY == MUBL_FAMILY_G4)

#ifndef MUBL_HSE_HZ
#error "FDCAN runs from HSE: set MUBL_HSE_HZ"
#endif
#ifndef MUBL_CAN_FD_DATA_BITRATE
#define MUBL_CAN_FD_DATA_BITRATE 0
#endif

#define FD MUBL_CAT3(FDCAN, MUBL_CAN_INSTANCE, _BASE)
#define R(off) REG32(FD + (off))
#if MUBL_FAMILY == MUBL_FAMILY_G4
/* Fixed G4 message RAM slice: 212 words (0x350 B) per FDCAN instance.
 * Offsets in 32-bit words: std filters 0, ext filters 28 (0x070), RXF0 44 (0x0B0), TX FIFO 158 (0x278). */
#define ELEMS 3U
#define ESIZE 18U
#define BASE_WORD ((MUBL_CAN_INSTANCE - 1U) * 212U)
#define FILTER_WORD (BASE_WORD + 28U)
#define RX_WORD (BASE_WORD + 44U)
#define TX_WORD (BASE_WORD + 158U)
#define CLEAR_WORDS 212U
#else
#define ELEMS 16U
#define ESIZE (MUBL_CAN_FD ? 18U : 4U)              /* words: 2 header + data */
#define BASE_WORD ((MUBL_CAN_INSTANCE - 1U) * 1280U) /* FDCAN2 in the second half of the RAM */
#define FILTER_WORD BASE_WORD
#define RX_WORD (BASE_WORD + 2U)
#define TX_WORD (RX_WORD + ELEMS * ESIZE)
#define CLEAR_WORDS (2U + 2U * ELEMS * ESIZE)
#endif
#define WORD(n) REG32(FDCAN_RAM + 4U * (n))
#define INIT_TIMEOUT 1000000U

static const uint8_t dlc_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

static bool wait_init(bool set)
{
    for (uint32_t i = 0; i < INIT_TIMEOUT; i++) {
        if (((R(FDCAN_CCCR) & FDCAN_CCCR_INIT) != 0U) == set) return true;
    }
    return false;
}

/* Configuration needs INIT and CCE; leaving INIT clears CCE. */
static bool configure(void)
{
    R(FDCAN_CCCR) |= FDCAN_CCCR_INIT;
    if (!wait_init(true)) return false;
    R(FDCAN_CCCR) |= FDCAN_CCCR_CCE;
    return true;
}

static void filter(uint32_t id, uint32_t mask)
{
    /* Mask 0 first, so in between the filter passes everything rather than something unintended. */
    WORD(FILTER_WORD + 1U) = 2U << 30;               /* EFT 10: classic id/mask filter */
    WORD(FILTER_WORD) = (1U << 29) | id;             /* EFEC 001: store in FIFO 0 */
    WORD(FILTER_WORD + 1U) = (2U << 30) | mask;
}

/* A bus-off sets INIT and leaves the controller off the bus until software clears it; clearing it
 * starts the standard recovery (128 x 11 recessive bits). */
static void recover(void)
{
    if (R(FDCAN_CCCR) & FDCAN_CCCR_INIT) R(FDCAN_CCCR) &= ~FDCAN_CCCR_INIT;
}

bool hal_can_start(uint32_t bitrate, bool listen_only)
{
    can_timing_t t;
    FDCAN_ENR |= FDCAN_BIT;
    FDCAN_RSTR |= FDCAN_BIT;
    FDCAN_RSTR &= ~FDCAN_BIT;
#if MUBL_FAMILY == MUBL_FAMILY_G4
    RCC_CCIPR &= ~(3U << RCC_CCIPR_FDCANSEL_POS);        /* kernel clock: HSE (00) */
#else
    RCC_D2CCIP1R &= ~(3U << RCC_D2CCIP1R_FDCANSEL_POS);  /* kernel clock: HSE */
#endif
    stm32_pin_af(MUBL_CAN_RX_PIN, false, true);
    stm32_pin_af(MUBL_CAN_TX_PIN, MUBL_CAN_TX_OPEN_DRAIN, false);
#ifdef MUBL_CAN_STANDBY_PIN
    stm32_pin_output(MUBL_CAN_STANDBY_PIN, !MUBL_CAN_STANDBY_LEVEL);
#endif
    if (!can_timing(MUBL_HSE_HZ, bitrate, 8, 25, 256, 128, 512, &t) || !configure()) return false;
    R(FDCAN_CCCR) = (R(FDCAN_CCCR) & ~(FDCAN_CCCR_MON | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE)) |
                    (listen_only ? FDCAN_CCCR_MON : 0U) | (MUBL_CAN_FD ? FDCAN_CCCR_FDOE : 0U) |
                    (MUBL_CAN_FD && MUBL_CAN_FD_DATA_BITRATE ? FDCAN_CCCR_BRSE : 0U);
    R(FDCAN_NBTP) = ((uint32_t)t.brp - 1U) << 16 | ((uint32_t)t.ts1 - 1U) << 8 | ((uint32_t)t.ts2 - 1U);
#if MUBL_CAN_FD && MUBL_CAN_FD_DATA_BITRATE
    if (!can_timing(MUBL_HSE_HZ, MUBL_CAN_FD_DATA_BITRATE, 5, 25, 32, 16, 32, &t)) return false;
    /* Transmitter delay compensation: at fast data rates the transceiver loop delay is a large part
     * of a bit, so the controller checks its own bits at a secondary sample point (ST's choice of
     * offset: prescaler x segment 1). */
    uint32_t tdco = (uint32_t)t.brp * t.ts1;
    if (tdco > 127U) tdco = 127U;
    R(FDCAN_DBTP) = FDCAN_DBTP_TDC | ((uint32_t)t.brp - 1U) << 16 | ((uint32_t)t.ts1 - 1U) << 8 |
                    ((uint32_t)t.ts2 - 1U) << 4;
    R(FDCAN_TDCR) = tdco << 8;
#endif
    for (uint32_t i = 0; i < CLEAR_WORDS; i++) WORD(BASE_WORD + i) = 0;
#if MUBL_FAMILY == MUBL_FAMILY_G4
    FDCAN_CKDIV = 0;                                 /* kernel clock / 1 (shared FDCAN_CONFIG) */
    R(FDCAN_TXBC) = 0;                               /* TX FIFO mode (TFQM = 0) */
    /* 1 extended filter (LSE=1), 0 standard filters, reject non-matching & remote frames. */
    R(FDCAN_RXGFC) = (1U << 24) | (2U << 4) | (2U << 2) | 3U;
    R(FDCAN_XIDAM) = 0x1FFFFFFFU;
#else
    R(FDCAN_SIDFC) = 0;
    R(FDCAN_XIDFC) = (1U << 16) | (FILTER_WORD << 2);
    R(FDCAN_XIDAM) = 0x1FFFFFFFU;
    R(FDCAN_RXF0C) = (ELEMS << 16) | (RX_WORD << 2);
    R(FDCAN_RXESC) = MUBL_CAN_FD ? 7U : 0U;
    R(FDCAN_TXBC) = (ELEMS << 24) | (TX_WORD << 2);  /* FIFO mode, no dedicated buffers */
    R(FDCAN_TXESC) = MUBL_CAN_FD ? 7U : 0U;
    R(FDCAN_GFC) = (2U << 4) | (2U << 2) | 3U;       /* reject non-matching and remote frames */
#endif
    hal_can_filter(0);
    R(FDCAN_CCCR) &= ~FDCAN_CCCR_INIT;
    (void)hal_can_error();
    return wait_init(false);
}

void hal_can_filter(uint8_t node_id)
{
    /* Extended frames pass the one filter element (accept everything until there is a node ID).
     * The element may be changed while running; doing that rather than re-entering configuration
     * mode keeps the FIFOs (setting CCE would empty them). */
    if (node_id == 0U) {
        filter(0, 0);
    } else {
        filter(((uint32_t)node_id << 8) | 0x80U, 0x7F80U);
    }
}

bool hal_can_receive(dc_frame_t* f)
{
    recover();
    for (;;) {
        const uint32_t s = R(FDCAN_RXF0S);
        if ((s & 0x7FU) == 0U) return false;
        const uint32_t gi = (s >> 8) & 0x3FU, e = RX_WORD + gi * ESIZE;
        const uint32_t r0 = WORD(e), r1 = WORD(e + 1U);
        const bool fdf = (r1 & (1U << 21)) != 0U;
        uint8_t len = dlc_len[(r1 >> 16) & 0xFU];
        if (!fdf && len > 8U) len = 8U;              /* classic frames: DLC 9-15 mean 8 bytes */
        const bool ok = (r0 & (1U << 30)) && !(r0 & (1U << 29)) && len <= DC_FRAME_MAX;
        if (ok) {
            f->id = r0 & 0x1FFFFFFFU;
            f->len = len;
            f->flags = fdf ? DC_FRAME_FLAG_FD : 0U;
            f->iface = 0;
            for (uint32_t i = 0; i < (len + 3U) / 4U; i++) mubl_put32(&f->data[4U * i], WORD(e + 2U + i));
        }
        R(FDCAN_RXF0A) = gi;
        if (ok) return true;
    }
}

bool hal_can_send(const dc_frame_t* f)
{
    recover();
    const uint32_t start = hal_millis();
    while (R(FDCAN_TXFQS) & (1U << 21)) {            /* TFQF: FIFO full */
        if (hal_millis() - start > MUBL_CAN_TX_TIMEOUT_MS) {
            R(FDCAN_TXBCR) = (1U << ELEMS) - 1U;     /* cancel everything pending */
            return false;
        }
    }
    uint8_t dlc = 0;
    while (dlc_len[dlc] < f->len) dlc++;
    const uint32_t pi = (R(FDCAN_TXFQS) >> 16) & 0x1FU, e = TX_WORD + pi * ESIZE;
    WORD(e) = (1U << 30) | (f->id & 0x1FFFFFFFU);    /* XTD */
    WORD(e + 1U) = ((uint32_t)dlc << 16) | (MUBL_CAN_FD ? (1U << 21) : 0U) |
                   (MUBL_CAN_FD && MUBL_CAN_FD_DATA_BITRATE ? (1U << 20) : 0U);
    uint8_t padded[64] = {0};
    memcpy(padded, f->data, f->len);
    for (uint32_t i = 0; i < (dlc_len[dlc] + 3U) / 4U; i++) WORD(e + 2U + i) = mubl_get32(&padded[4U * i]);
    R(FDCAN_TXBAR) = 1U << pi;
    return true;
}

bool hal_can_error(void)
{
    const uint32_t lec = R(FDCAN_PSR) & 7U;          /* reading PSR sets LEC back to 7 */
    return lec != 0U && lec != 7U;
}

void hal_can_stop(void)
{
    FDCAN_RSTR |= FDCAN_BIT;
    FDCAN_RSTR &= ~FDCAN_BIT;
    FDCAN_ENR &= ~FDCAN_BIT;
}

#endif
