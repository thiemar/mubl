/*
 * The host side of the simulated USB peripherals (usb_fs_model.c for the F0/F3 device peripheral,
 * usb_otg_model.c for the OTG core: one is linked, by MUBL_USB_OTG_CORE), used by the simulated
 * host (usb_host.c) and the driver tests.  Each call is one bus transaction.
 */
#ifndef MUBL_USB_MODEL_H
#define MUBL_USB_MODEL_H

#include <stdbool.h>
#include <stdint.h>

/* Clock and reset, as the driver's STM32 code does them: false if there is no 48 MHz clock. */
bool hw_start(void);
void hw_stop(void);              /* peripheral reset and clock off: detached, nothing answers */

/* Host side: one call is one transaction, returning the device's handshake */
typedef enum { USBM_ACK, USBM_NAK, USBM_STALL, USBM_TIMEOUT } usbm_handshake_t;

void usbm_reset_all(void);        /* power-on state: unpowered, no errors */
void usbm_bus_reset(void);
void usbm_sof(void);              /* a start-of-frame packet: sets ISTR.SOF */
usbm_handshake_t usbm_setup(uint8_t address, const uint8_t packet[8]);
usbm_handshake_t usbm_out(uint8_t address, uint8_t ep, const uint8_t *data, uint16_t len);
usbm_handshake_t usbm_in(uint8_t address, uint8_t ep, uint8_t *data, uint16_t *len);

/* Host resets its expected data toggle (after SET_CONFIGURATION or
 * CLEAR_FEATURE(ENDPOINT_HALT)) */
void usbm_reset_host_toggle(uint8_t ep_address);

bool usbm_connected(void);      /* powered and D+ pull-up on */
bool usbm_take_detach(void);     /* the pull-up went off since the last call (a reset, a detach) */
uint8_t usbm_daddr(void);

/* Protocol violations detected by the model since the last call */
int usbm_take_errors(void);

/* Count violations without printing them (for deliberate negative tests) */
void usbm_set_quiet(bool quiet);

#endif
