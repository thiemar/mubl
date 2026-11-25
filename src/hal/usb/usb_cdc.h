/*
 * USB CDC-ACM device for the serial protocol's USB link, polled (the USB interrupt is never used).
 *
 * usb_cdc.c holds what does not depend on the peripheral: descriptors, the standard (USB 2.0
 * chapter 9) and CDC class requests, and the CDC line state.  A device layer per peripheral moves
 * the packets and implements hal_usb_* (hal.h):
 *
 *   usb_fs.c    the full-speed device peripheral with packet memory (F0, F3), after µCAN's driver
 *   usb_otg.c   the OTG full-speed core (F4, H7)
 *
 * One fixed configuration: EP0 control, EP1 bulk IN (0x81), EP2 bulk OUT (0x02) and EP3 interrupt
 * IN (0x83, CDC notifications, never sent).  The CDC interface is at device level (class 2), so
 * hosts bind their serial-port driver without an interface association descriptor.
 */
#ifndef MUBL_USB_CDC_H
#define MUBL_USB_CDC_H

#include "mubl.h"

#define USB_EP0_SIZE 64U
#define USB_DATA_SIZE 64U
#define USB_NOTIFY_SIZE 8U

#define USB_EP_IN 1U
#define USB_EP_OUT 2U
#define USB_EP_NOTIFY 3U
#define USB_EP_IN_ADDR (0x80U | USB_EP_IN)
#define USB_EP_OUT_ADDR USB_EP_OUT
#define USB_EP_NOTIFY_ADDR (0x80U | USB_EP_NOTIFY)

#define USB_REQ_DIR_IN 0x80U

typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

/* ---- usb_cdc.c, for the device layer ------------------------------------------------------ */

extern usb_setup_t usb_req;                     /* the control request being handled */
extern uint8_t usb_ctrl_buf[USB_EP0_SIZE];      /* generated replies and OUT data stages */
extern uint8_t usb_configuration;               /* 0: not configured */

/* A SETUP packet arrived (in usb_req): answer it through the usbd_* calls below.  False: the
 * request is not supported and the device layer stalls it. */
bool usb_setup(void);
/* The OUT data stage the request asked for (usbd_receive) is in usb_ctrl_buf.  False: stall. */
bool usb_setup_out(void);
/* Bus reset or deconfiguration: forget the line state. */
void usb_cdc_reset(void);

/* ---- The device layer, for usb_cdc.c ------------------------------------------------------- */

/* Send `len` bytes (clipped to wLength) as the data stage of usb_req, or acknowledge a request
 * with no data stage.  `data` must stay valid until it is sent (usb_ctrl_buf or constant). */
void usbd_reply(const uint8_t* data, uint16_t len);
void usbd_ack(void);                            /* zero-length status stage */
bool usbd_receive(uint16_t len);                /* expect an OUT data stage into usb_ctrl_buf */
void usbd_set_address(uint8_t address);         /* acknowledge, and apply as the hardware requires */
void usbd_configure(uint8_t value);             /* data endpoints on (1) or off (0), buffers emptied */
int usbd_halted(uint16_t ep_address);           /* 1 halted, 0 not, -1 no such endpoint */
bool usbd_halt(uint16_t ep_address, bool halt); /* SET/CLEAR_FEATURE(ENDPOINT_HALT) */

/* Transmit ring, for hal_usb_stop and tests: room for hal_usb_write (0 if not configured), and
 * whether everything written has been collected by the host. */
uint16_t usbd_tx_space(void);
bool usbd_tx_empty(void);

#endif
