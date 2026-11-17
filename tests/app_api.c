/* app/mubl_app.h compiled for the host, so tests can drive the simulation the way an application does. */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint8_t block[256];
#define MUBL_HANDOFF_ADDR ((uintptr_t)block)
#include "mubl_app.h"

uint8_t* app_block(void) { return block; }
void app_load(const uint8_t* data) { memcpy(block, data, sizeof block); }
void app_request(uint8_t link, uint8_t node, uint32_t bitrate, uint8_t server, const char* path, uint32_t baud,
                 uint8_t srxl2)
{
    mubl_app_request(link, node, bitrate, server, path, baud, srxl2);
}
int app_info(mubl_info_t* out) { return mubl_app_info(out); }
void app_confirm(void) { mubl_app_confirm(); }
