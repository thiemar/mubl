/*
 * Host-side model of the STM32F0/F3 full-speed USB device peripheral, from µCAN's test/usb_sim.c:
 * src/hal/usb/usb_fs.c runs against it in the simulation and in tests/usb_fs_test.c.  The model
 * is written from RM0091 and is independent of the driver's own register definitions.
 *
 * It covers what the driver depends on: EPnR write semantics (rw, toggle and
 * rc_w0 bits), ISTR, the packet memory and buffer descriptor table, address
 * filtering, NAK/STALL handshakes and DATA0/DATA1 toggling. It does not model
 * timing, suspend/resume or electrical behaviour.
 */

#ifndef MUBL_USB_FS_MODEL_H
#define MUBL_USB_FS_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#include "usb_model.h"

/* Hardware interface consumed by src/hal/usb/usb_fs.c in the simulation */
uint16_t hw_epr_get(uint8_t ep);
void hw_epr_set(uint8_t ep, uint16_t value);
uint16_t hw_istr_get(void);
void hw_istr_clear(uint16_t bits);
volatile uint16_t *hw_pma(uint16_t offset);
void hw_set_address(uint8_t address);
void hw_connect(void);
void hw_disconnect(void);
uint32_t hw_irq_save(void);
void hw_irq_restore(uint32_t mask);

/* Run `fn` the next time the driver writes EPnR `ep`, between its read and
 * its write -- i.e. a bus transaction completing mid read-modify-write */
void usbm_before_next_write(uint8_t ep, void (*fn)(void));
bool usbm_irq_masked(void);

#endif /* MUBL_USB_FS_MODEL_H */
