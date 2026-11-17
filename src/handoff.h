#ifndef MUBL_HANDOFF_H
#define MUBL_HANDOFF_H

#include "mubl.h"

#define MUBL_HANDOFF_TO_BL 0x4C42756DU    /* "muBL": written by the application */
#define MUBL_HANDOFF_TO_APP 0x5041756DU   /* "muAP": written by the bootloader */

/* Read (and consume) whatever the application left: the native block, then the compatibility
 * structures that are enabled. */
void handoff_read(mubl_request_t* req);
/* Leave the session details for the application; called after hal_deinit, just before the jump. */
void handoff_write(const mubl_session_t* s, uint8_t boot_attempts);

#if MUBL_COMPAT_PX4_CAN
/* PX4 shared structure in CAN1 filter banks 2-4: {crc_hi, crc_lo, signature, bus_speed, node_id}.
 * Writing NULL clears it. */
void hal_px4_shared_read(uint32_t w[5]);
void hal_px4_shared_write(const uint32_t w[5]);
#endif

#endif
