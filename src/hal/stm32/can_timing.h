/*
 * Bit timing: the largest number of time quanta per bit (min_tq..max_tq) that divides the kernel
 * clock exactly, sample point as close to 87.5 % as the segments allow, SJW 1.
 */
#ifndef MUBL_CAN_TIMING_H
#define MUBL_CAN_TIMING_H

#include "mubl.h"

typedef struct {
    uint16_t brp;
    uint8_t ts1, ts2;         /* time quanta, not register values */
} can_timing_t;

static inline bool can_timing(uint32_t clock, uint32_t bitrate, uint32_t min_tq, uint32_t max_tq,
                              uint32_t max_ts1, uint32_t max_ts2, uint32_t max_brp, can_timing_t* t)
{
    for (uint32_t tq = max_tq; tq >= min_tq; tq--) {
        const uint32_t per = bitrate * tq;
        if (clock % per != 0U || clock / per > max_brp) continue;
        uint32_t ts1 = (tq * 7U + 4U) / 8U - 1U;     /* sync segment + ts1 = 87.5 % of the bit */
        if (ts1 > max_ts1) ts1 = max_ts1;
        const uint32_t ts2 = tq - 1U - ts1;
        if (ts2 < 1U || ts2 > max_ts2) continue;
        t->brp = (uint16_t)(clock / per);
        t->ts1 = (uint8_t)ts1;
        t->ts2 = (uint8_t)ts2;
        return true;
    }
    return false;
}

#endif
