/* The simulated USB host (usb_host.c), driven by hal_host.c. */
#ifndef MUBL_USB_HOST_H
#define MUBL_USB_HOST_H

#include <stdint.h>

/* `step` runs the device's main loop once; it returns 0 once the bootloader has stopped. */
void usb_host_reset(int (*step)(void));
void usb_host_frame(uint32_t now);     /* every millisecond: attach, enumeration, start-of-frame */
void usb_host_poll(void);              /* before each device step: bulk transfers */

#endif
