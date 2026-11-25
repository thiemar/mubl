/*
 * µBL application interface (header only, no dependencies beyond <stdint.h> and <stdbool.h>).
 *
 * The bootloader and the application talk through a block of retained RAM.  Before including this
 * header define MUBL_HANDOFF_ADDR to the bootloader board's value (by default the start of RAM,
 * after 0xC0 bytes on Cortex-M0 and after AP_Periph's 256-byte structure when that compatibility
 * is enabled; `make BOARD=x` prints nothing about it, so read src/mubl_config.h with the board's
 * settings).  The application's linker script must keep that RAM out of .data, .bss and the stack
 * (see app/README.md).
 *
 *   mubl_app_info(&info)    what the bootloader passed: the link it used, CAN node ID and bitrate,
 *                           UART baud rate, SRXL2 device ID, and the boot attempt count
 *   mubl_app_confirm()      this boot is good: clears the attempt count (MUBL_MAX_BOOT_ATTEMPTS)
 *   mubl_app_request(...)   ask the bootloader to stay after the next reset, optionally on one link
 *                           and, for DroneCAN, to fetch an image from a file server at once; then
 *                           reset the MCU (NVIC_SystemReset)
 *
 * Call mubl_app_info early (before anything can overwrite the block), then mubl_app_confirm once
 * the application is known to run.
 */
#ifndef MUBL_APP_H
#define MUBL_APP_H

#include <stdbool.h>
#include <stdint.h>

#ifndef MUBL_HANDOFF_ADDR
#error "define MUBL_HANDOFF_ADDR to the bootloader board's handoff address"
#endif

#define MUBL_HANDOFF_TO_BL 0x4C42756DU
#define MUBL_HANDOFF_TO_APP 0x5041756DU
#define MUBL_LINK_ANY 0
#define MUBL_LINK_DRONECAN 1
#define MUBL_LINK_SERIAL 2
#define MUBL_LINK_SRXL2 3
#define MUBL_LINK_USB 4

typedef struct {
    uint8_t link;              /* MUBL_LINK_*: the link the bootloader last used, 0 if none */
    uint8_t boot_attempts;     /* boots not yet confirmed, this one included */
    uint8_t can_node_id;       /* 0 if DroneCAN did not get one */
    uint8_t srxl2_device_id;
    uint32_t can_bitrate;      /* 0 if not detected */
    uint32_t uart_baud;
} mubl_info_t;

static inline uint8_t* mubl__block(void)
{
    return (uint8_t*)(uintptr_t)(MUBL_HANDOFF_ADDR);
}

static inline uint32_t mubl__get32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void mubl__put32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline uint32_t mubl__crc32(const uint8_t* p, uint32_t n)   /* zlib CRC-32 */
{
    uint32_t crc = 0xFFFFFFFFU;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

static inline bool mubl_app_info(mubl_info_t* out)
{
    const uint8_t* h = mubl__block();
    const uint16_t len = (uint16_t)(h[8] | (h[9] << 8));
    if (mubl__get32(h) != MUBL_HANDOFF_TO_APP || len < 20U || len > 228U || h[10] != 1U ||
        mubl__crc32(h + 8, len) != mubl__get32(h + 4)) {
        return false;
    }
    out->link = h[12];
    out->boot_attempts = h[13];
    out->can_node_id = h[14];
    out->can_bitrate = mubl__get32(h + 16);
    out->uart_baud = mubl__get32(h + 20);
    out->srxl2_device_id = h[24];
    return true;
}

static inline void mubl_app_confirm(void)
{
    mubl__put32(mubl__block(), 0);
}

/* path may be NULL; it is used only with can_server_node_id (an image on a DroneCAN file server). */
static inline void mubl_app_request(uint8_t link, uint8_t can_node_id, uint32_t can_bitrate,
                                    uint8_t can_server_node_id, const char* path, uint32_t uart_baud,
                                    uint8_t srxl2_device_id)
{
    uint8_t* h = mubl__block();
    uint8_t n = 0;
    while (path != 0 && path[n] != '\0' && n < 200U) {
        h[28 + n] = (uint8_t)path[n];
        n++;
    }
    h[8] = (uint8_t)(20U + n);
    h[9] = 0;
    h[10] = 1;                 /* version */
    h[11] = 1;                 /* stay in the bootloader */
    h[12] = link;
    h[13] = 0;
    h[14] = can_node_id;
    h[15] = can_server_node_id;
    mubl__put32(h + 16, can_bitrate);
    mubl__put32(h + 20, uart_baud);
    h[24] = srxl2_device_id;
    h[25] = h[26] = 0;
    h[27] = n;
    mubl__put32(h + 4, mubl__crc32(h + 8, 20U + n));
    mubl__put32(h, MUBL_HANDOFF_TO_BL);
}

#endif /* MUBL_APP_H */
