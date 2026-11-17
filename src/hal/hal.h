/*
 * Hardware abstraction.  One implementation per MCU family (src/hal/stm32) plus the host
 * simulation (src/hal/host).  Everything is polled; the only interrupt is the millisecond tick.
 */
#ifndef MUBL_HAL_H
#define MUBL_HAL_H

#include "mubl.h"

/* ---- System ------------------------------------------------------------------------------- */

void hal_init(void);                       /* clocks, tick, watchdog, board pins */
uint32_t hal_millis(void);
void hal_delay_us(uint32_t us);
void hal_watchdog_refresh(void);
void hal_unique_id(uint8_t out[16]);       /* MCU unique ID, zero padded to 16 bytes */
uint32_t hal_idcode(void);                 /* DBGMCU IDCODE */
bool hal_hold_pin(void);                   /* the board's "stay in bootloader" input is asserted */
void hal_led(bool on);
__attribute__((noreturn)) void hal_reset(void);

/* Leave the bootloader: hal_deinit returns the peripherals and clocks to their reset state (the
 * handoff is written after it), then hal_start_app jumps. */
void hal_deinit(void);
__attribute__((noreturn)) void hal_start_app(void);

/* Run fn(arg) and return true, or return false if it raised a bus fault (an uncorrectable flash
 * ECC error).  Without MUBL_FLASH_ECC this just calls fn. */
bool hal_guarded(void (*fn)(void*), void* arg);

/* ---- Flash (absolute addresses inside the application region) ------------------------------ */

/* The erase unit containing addr: [*start, *end). */
void hal_flash_sector(uint32_t addr, uint32_t* start, uint32_t* end);
bool hal_flash_erase(uint32_t start);
/* Program one MUBL_FLASH_GRANULE-byte unit at a granule-aligned, erased address. */
bool hal_flash_program(uint32_t addr, const uint8_t* data);

/* ---- CAN (one interface) ------------------------------------------------------------------ */

#if MUBL_DRONECAN
#include "dronecan.h"

/* (Re)start the controller.  Listen-only sends nothing, not even ACKs (autobaud).  Until
 * hal_can_filter is called every extended data frame is received. */
bool hal_can_start(uint32_t bitrate, bool listen_only);
/* Receive only service frames addressed to node_id (node_id 0: everything again). */
void hal_can_filter(uint8_t node_id);
bool hal_can_receive(dc_frame_t* frame);
bool hal_can_send(const dc_frame_t* frame);   /* waits up to MUBL_CAN_TX_TIMEOUT_MS for room */
/* True once since the last call if the controller saw a protocol error (wrong bitrate). */
bool hal_can_error(void);
void hal_can_stop(void);
#endif

/* ---- UART ports (0, 1) -------------------------------------------------------------------- */

#if MUBL_SERIAL || MUBL_SRXL2
void hal_uart_start(uint8_t port, uint32_t baud);
int hal_uart_getc(uint8_t port);           /* a received byte, or -1 */
void hal_uart_write(uint8_t port, const uint8_t* data, uint16_t n);  /* returns when sent */
void hal_uart_stop(uint8_t port);
#endif

#endif /* MUBL_HAL_H */
