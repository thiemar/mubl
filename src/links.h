/*
 * Link protocols and the UART port sharing between them.
 */
#ifndef MUBL_LINKS_H
#define MUBL_LINKS_H

#include "mubl.h"

/* Each protocol: start listening, run its timers, stop (and go silent). */
#if MUBL_DRONECAN
void dronecan_start(void);
void dronecan_poll(uint32_t now);
void dronecan_stop(void);
#endif
#if MUBL_SERIAL
void serial_start(void);
void serial_poll(uint32_t now);
void serial_rx(uint8_t byte, uint32_t now);
void serial_stop(void);
#endif
#if MUBL_SRXL2
void srxl2_start(void);
void srxl2_poll(uint32_t now);
void srxl2_rx(uint8_t byte, uint32_t now);
void srxl2_stop(void);
#endif

/* Position of a link in MUBL_LINK_PRIORITY (0 is the highest), or 0xFF if it is not listed. */
uint8_t mubl_rank(uint8_t link);

#if MUBL_SERIAL || MUBL_SRXL2
/*
 * UART port sharing.  Protocols on the same port all see its received bytes until one of them
 * decodes a valid frame and claims the port; from then on only the owner hears it.  If they
 * listen at different baud rates the port alternates between the rates every MUBL_UART_SLICE_MS,
 * in priority order, until it is claimed.  Before a claim only the highest-priority listener may
 * transmit (SRXL2's start-up handshake is the only unsolicited transmission).
 */
void uart_mux_init(void);
void uart_listen(uint8_t link, uint8_t port, uint32_t baud);
void uart_unlisten(uint8_t link);
void uart_mux_poll(uint32_t now);
bool uart_claim(uint8_t link);           /* false if another link owns the port */
bool uart_may_send(uint8_t link);
void uart_send(uint8_t link, const uint8_t* data, uint16_t n);
void uart_set_baud(uint8_t link, uint32_t baud);   /* the owner only */
uint32_t uart_baud(uint8_t link);
#endif

#endif
