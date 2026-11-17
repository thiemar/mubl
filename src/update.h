#ifndef MUBL_UPDATE_H
#define MUBL_UPDATE_H

#include "mubl.h"

/* Offsets are relative to the start of the application region.  `owner` is the MUBL_LINK_* of
 * the link driving the update; calls from any other link get MUBL_E_BUSY while it is active. */

void update_init(void);
bool update_active(void);
uint8_t update_owner(void);                /* 0 when idle */
uint32_t update_position(void);            /* bytes written so far */

/* Start an update (again, for the same owner): erases the first sector, which makes the current
 * application unbootable. */
int update_begin(uint8_t owner);
/* update_begin, then erase the rest of the region (the serial protocol's CHIP_ERASE). */
int update_erase_all(uint8_t owner);
/* Append image bytes; offset must equal update_position(). */
int update_write(uint8_t owner, uint32_t offset, const uint8_t* data, uint32_t n);
/* Validate the image and, if it passes, make it bootable.  The update ends either way. */
int update_finish(uint8_t owner);
void update_abort(uint8_t owner);

/* CRC-32 (crc32_update convention) of region bytes [from, to) as they will be when the update
 * finishes.  False on a flash read error or a bad range. */
bool update_crc32(uint32_t crc, uint32_t from, uint32_t to, uint32_t* out);

#endif
