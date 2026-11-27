/*
 * bxCAN (F0, F3, F4): FIFO 0 receive, three transmit mailboxes sent in request order (TXFP) so a
 * multi-frame transfer keeps its frame order, automatic bus-off recovery, polled.
 *
 * Filters use the board's first bank (CAN1_FIRST_BANK / CAN2_FIRST_BANK) only, leaving banks 2-4
 * to the PX4 handoff, which hal_px4_shared_* reads and writes in CAN1's filter registers.
 */
#include "can_timing.h"
#include "hal/hal.h"
#include "stm32.h"

#if MUBL_DRONECAN && MUBL_FAMILY != MUBL_FAMILY_H7 && MUBL_FAMILY != MUBL_FAMILY_G4

#define BX MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _BASE)
#define CAN_ENR MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _ENR)
#define CAN_RSTR MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _RSTR)
#define CAN_BIT MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _BIT)
#define CAN_CLK MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _CLK)
#define FILTERS MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _FILTER_BASE)
#define BANK MUBL_CAT3(CAN, MUBL_CAN_INSTANCE, _FIRST_BANK)
#define R(off) REG32(BX + (off))
#define F(off) REG32(FILTERS + (off))
#define INAK_TIMEOUT 1000000U

MUBL_STATIC_ASSERT(!MUBL_CAN_FD, "bxCAN has no CAN FD");
MUBL_STATIC_ASSERT(BANK != 2 && BANK != 3 && BANK != 4, "filter banks 2-4 hold the PX4 handoff");

static bool wait_inak(bool set)
{
    for (uint32_t i = 0; i < INAK_TIMEOUT; i++) {
        if (((R(CAN_MSR) & CAN_MSR_INAK) != 0U) == set) return true;
    }
    return false;
}

/* One 32-bit mask filter for extended data frames: (frame ID & mask) == id. */
static void filter(uint32_t id, uint32_t mask)
{
    const uint32_t bit = 1U << BANK;
    F(CAN_FMR) |= CAN_FMR_FINIT;
    F(CAN_FA1R) &= ~bit;
    F(CAN_FS1R) |= bit;                              /* 32-bit */
    F(CAN_FM1R) &= ~bit;                             /* mask mode */
    F(CAN_FFA1R) &= ~bit;                            /* FIFO 0 */
    F(CAN_FR1(BANK)) = (id << 3) | CAN_RIR_IDE;
    F(CAN_FR2(BANK)) = (mask << 3) | CAN_RIR_IDE | CAN_RIR_RTR;
    F(CAN_FA1R) |= bit;
    F(CAN_FMR) &= ~CAN_FMR_FINIT;
}

bool hal_can_start(uint32_t bitrate, bool listen_only)
{
    can_timing_t t;
    CAN_ENR |= CAN_BIT;
    CAN_RSTR |= CAN_BIT;
    CAN_RSTR &= ~CAN_BIT;
    stm32_pin_af(MUBL_CAN_RX_PIN, false, true);
    stm32_pin_af(MUBL_CAN_TX_PIN, false, false);
#ifdef MUBL_CAN_STANDBY_PIN
    stm32_pin_output(MUBL_CAN_STANDBY_PIN, !MUBL_CAN_STANDBY_LEVEL);
#endif
#if MUBL_FAMILY == MUBL_FAMILY_F0 && defined(MUBL_F0_PA11_PA12_REMAP)
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
#endif
    if (!can_timing(CAN_CLK, bitrate, 8, 20, 16, 8, 1024, &t)) return false;
    R(CAN_MCR) = CAN_MCR_INRQ;                       /* leaves sleep mode */
    if (!wait_inak(true)) return false;
    R(CAN_MCR) = CAN_MCR_INRQ | CAN_MCR_TXFP | CAN_MCR_ABOM;
    R(CAN_BTR) = (listen_only ? CAN_BTR_SILM : 0U) | ((uint32_t)(t.ts2 - 1U) << 20) |
                 ((uint32_t)(t.ts1 - 1U) << 16) | (t.brp - 1U);
    filter(0, 0);
    R(CAN_MCR) &= ~CAN_MCR_INRQ;
    (void)hal_can_error();
    return wait_inak(false);                         /* needs 11 recessive bits */
}

void hal_can_filter(uint8_t node_id)
{
    if (node_id == 0U) {
        filter(0, 0);
    } else {
        filter(((uint32_t)node_id << 8) | 0x80U, 0x7F80U);   /* service frames to node_id */
    }
}

bool hal_can_receive(dc_frame_t* f)
{
    for (;;) {
        if ((R(CAN_RF0R) & CAN_RF0R_FMP0) == 0U) {
            if (R(CAN_RF0R) & CAN_RF0R_FOVR0) R(CAN_RF0R) = CAN_RF0R_FOVR0;
            return false;
        }
        const uint32_t rir = R(CAN_RI0R), rdtr = R(CAN_RDT0R);
        const uint32_t lo = R(CAN_RDL0R), hi = R(CAN_RDH0R);
        R(CAN_RF0R) = CAN_RF0R_RFOM0;
        if (!(rir & CAN_RIR_IDE) || (rir & CAN_RIR_RTR)) continue;
        f->id = rir >> 3;
        f->len = (uint8_t)((rdtr & 0xFU) > 8U ? 8U : (rdtr & 0xFU));
        f->flags = 0;
        f->iface = 0;
        mubl_put32(f->data, lo);
        mubl_put32(f->data + 4, hi);
        return true;
    }
}

bool hal_can_send(const dc_frame_t* f)
{
    const uint32_t start = hal_millis();
    while ((R(CAN_TSR) & CAN_TSR_TME_ALL) == 0U) {
        if (hal_millis() - start > MUBL_CAN_TX_TIMEOUT_MS) {
            /* Nobody acknowledges (alone on the bus, or bus off): drop what is queued. */
            R(CAN_TSR) = CAN_TSR_ABRQ(0) | CAN_TSR_ABRQ(1) | CAN_TSR_ABRQ(2);
            return false;
        }
    }
    const uint32_t mb = (R(CAN_TSR) >> CAN_TSR_CODE_POS) & 3U;
    R(CAN_TDTR(mb)) = f->len;
    R(CAN_TDLR(mb)) = mubl_get32(f->data);
    R(CAN_TDHR(mb)) = mubl_get32(f->data + 4);
    R(CAN_TIR(mb)) = (f->id << 3) | CAN_TIR_IDE | CAN_TIR_TXRQ;
    return true;
}

bool hal_can_error(void)
{
    const uint32_t lec = (R(CAN_ESR) >> CAN_ESR_LEC_POS) & 7U;
    R(CAN_ESR) = 7U << CAN_ESR_LEC_POS;              /* "set by software": changes on the next event */
    return lec != 0U && lec != 7U;
}

void hal_can_stop(void)
{
    CAN_RSTR |= CAN_BIT;
    CAN_RSTR &= ~CAN_BIT;
    CAN_ENR &= ~CAN_BIT;
}

#if MUBL_COMPAT_PX4_CAN
/* PX4 boot_app_shared: crc_hi FIR(2,1), crc_lo FIR(2,2), signature FIR(3,1), bus_speed FIR(3,2),
 * node_id FIR(4,1), in CAN1's filter banks. */
void hal_px4_shared_read(uint32_t w[5])
{
    RCC_APB1ENR |= CAN1_BIT;
    w[0] = REG32(CAN1_BASE + CAN_FR1(2));
    w[1] = REG32(CAN1_BASE + CAN_FR2(2));
    w[2] = REG32(CAN1_BASE + CAN_FR1(3));
    w[3] = REG32(CAN1_BASE + CAN_FR2(3));
    w[4] = REG32(CAN1_BASE + CAN_FR1(4));
}

void hal_px4_shared_write(const uint32_t w[5])
{
    static const uint32_t zero[5];
    if (w == NULL) w = zero;
    RCC_APB1ENR |= CAN1_BIT;
    REG32(CAN1_BASE + CAN_FMR) |= CAN_FMR_FINIT;
    REG32(CAN1_BASE + CAN_FR1(2)) = w[0];
    REG32(CAN1_BASE + CAN_FR2(2)) = w[1];
    REG32(CAN1_BASE + CAN_FR1(3)) = w[2];
    REG32(CAN1_BASE + CAN_FR2(3)) = w[3];
    REG32(CAN1_BASE + CAN_FR1(4)) = w[4];
    REG32(CAN1_BASE + CAN_FMR) &= ~CAN_FMR_FINIT;
}
#endif

#endif
