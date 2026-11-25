/*
 * Host-side model of the USB OTG core (Synopsys DWC2) in device mode with the embedded full-speed
 * PHY, as on STM32F4 (OTG_FS) and H7: src/hal/usb/usb_otg.c runs against it in the simulation
 * (MUBL_USB_OTG_CORE=1) and in tests/usb_fs_test.c.  Written from RM0090/RM0433 and independent of
 * the driver's register definitions.
 *
 * It covers what the driver depends on: core reset and FIFO flushes, device mode and the session
 * (VBUS sensing off: NOVBUSSENS on the F405 core, the B-session override on cores with VBDEN, as
 * STM32_OTG_VBDEN selects), soft disconnect, the shared receive FIFO and its status entries,
 * per-endpoint transmit FIFOs and their sizes, endpoint enable, disable, NAK and STALL, transfer
 * size and packet counts, SETUP packet counting, data toggles, the device address and the
 * start-of-frame, reset and enumeration-done flags.  Not timing, suspend, or the interrupt line.
 */
#ifndef MUBL_USB_OTG_MODEL_H
#define MUBL_USB_OTG_MODEL_H

#include "usb_model.h"

uint32_t otgm_rd(uint32_t offset);
void otgm_wr(uint32_t offset, uint32_t value);
#define rd otgm_rd
#define wr otgm_wr

#endif
