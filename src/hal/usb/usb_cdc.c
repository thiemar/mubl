/*
 * USB descriptors, standard and CDC-ACM class requests (see usb_cdc.h).  Shared by the device
 * layers; nothing here touches hardware.
 */
#include "usb_cdc.h"

#include "hal/hal.h"

#if MUBL_USB

#define LO(x) ((uint8_t)((x) & 0xFFU))
#define HI(x) ((uint8_t)(((x) >> 8) & 0xFFU))

static const uint8_t device_desc[18] = {
    18, 1,               /* bLength, DEVICE */
    0x00, 0x02,          /* bcdUSB 2.00 */
    0x02, 0x00, 0x00,    /* CDC at device level, so hosts bind without an IAD */
    USB_EP0_SIZE,
    LO(MUBL_USB_VID), HI(MUBL_USB_VID),
    LO(MUBL_USB_PID), HI(MUBL_USB_PID),
    LO(MUBL_USB_BCD_DEVICE), HI(MUBL_USB_BCD_DEVICE),
    1, 2, 3,             /* iManufacturer, iProduct, iSerialNumber */
    1                    /* bNumConfigurations */
};

#define CONFIG_DESC_LEN 67U

static const uint8_t config_desc[CONFIG_DESC_LEN] = {
    9, 2, LO(CONFIG_DESC_LEN), HI(CONFIG_DESC_LEN),
    2,                   /* bNumInterfaces */
    1,                   /* bConfigurationValue */
    0,                   /* iConfiguration */
    0x80,                /* bmAttributes: bus powered */
    MUBL_USB_MAX_POWER_MA / 2U,

    /* Interface 0: communications class, abstract control model */
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,       /* header, CDC 1.10 */
    5, 0x24, 0x01, 0x00, 1,          /* call management: none, data interface 1 */
    4, 0x24, 0x02, 0x02,             /* ACM: line coding and control line state */
    5, 0x24, 0x06, 0, 1,             /* union: control 0, data 1 */
    7, 5, USB_EP_NOTIFY_ADDR, 0x03, LO(USB_NOTIFY_SIZE), HI(USB_NOTIFY_SIZE), 16,

    /* Interface 1: CDC data */
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, USB_EP_OUT_ADDR, 0x02, LO(USB_DATA_SIZE), HI(USB_DATA_SIZE), 0,
    7, 5, USB_EP_IN_ADDR, 0x02, LO(USB_DATA_SIZE), HI(USB_DATA_SIZE), 0
};

#define REQ_TYPE_MASK 0x60U
#define REQ_TYPE_STANDARD 0x00U
#define REQ_TYPE_CLASS 0x20U
#define REQ_RECIP_MASK 0x1FU
#define REQ_RECIP_DEVICE 0U
#define REQ_RECIP_INTERFACE 1U
#define REQ_RECIP_ENDPOINT 2U

#define REQ_GET_STATUS 0U
#define REQ_CLEAR_FEATURE 1U
#define REQ_SET_FEATURE 3U
#define REQ_SET_ADDRESS 5U
#define REQ_GET_DESCRIPTOR 6U
#define REQ_GET_CONFIGURATION 8U
#define REQ_SET_CONFIGURATION 9U
#define REQ_GET_INTERFACE 10U
#define REQ_SET_INTERFACE 11U

#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
#define CDC_SEND_BREAK 0x23U

#define DESC_DEVICE 1U
#define DESC_CONFIGURATION 2U
#define DESC_STRING 3U

usb_setup_t usb_req;
uint8_t usb_ctrl_buf[USB_EP0_SIZE];
uint8_t usb_configuration;
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};   /* 115200 8N1: not used */

void usb_cdc_reset(void)
{
    usb_configuration = 0;
}

/* ASCII text, or the unique ID in hex (index 3), as a string descriptor in usb_ctrl_buf. */
static uint16_t string_descriptor(uint8_t index)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t* out = usb_ctrl_buf;
    uint16_t len = 2;
    const char* text = NULL;
    switch (index) {
    case 0:
        out[2] = 0x09;                   /* English (United States) */
        out[3] = 0x04;
        len = 4;
        break;
    case 1: text = MUBL_USB_MANUFACTURER; break;
    case 2: text = MUBL_USB_PRODUCT; break;
    case 3: {
        uint8_t uid[16];
        hal_unique_id(uid);
        for (uint8_t i = 0; i < 24U; i++) {
            out[len++] = (uint8_t)hex[(uid[i / 2U] >> ((i & 1U) ? 0U : 4U)) & 0xFU];
            out[len++] = 0;
        }
        break;
    }
    default:
        return 0;
    }
    while (text != NULL && *text != '\0' && len + 2U <= USB_EP0_SIZE) {
        out[len++] = (uint8_t)*text++;
        out[len++] = 0;
    }
    out[0] = (uint8_t)len;
    out[1] = DESC_STRING;
    return len;
}

static bool get_descriptor(void)
{
    const uint8_t type = HI(usb_req.wValue), index = LO(usb_req.wValue);
    switch (type) {
    case DESC_DEVICE:
        if (index != 0U) return false;
        usbd_reply(device_desc, sizeof device_desc);
        return true;
    case DESC_CONFIGURATION:
        if (index != 0U) return false;
        usbd_reply(config_desc, sizeof config_desc);
        return true;
    case DESC_STRING: {
        const uint16_t len = string_descriptor(index);
        if (len == 0U) return false;
        usbd_reply(usb_ctrl_buf, len);
        return true;
    }
    default:
        return false;                    /* DEVICE_QUALIFIER included: full speed only */
    }
}

static bool standard_request(void)
{
    const usb_setup_t* r = &usb_req;
    const uint8_t recipient = r->bmRequestType & REQ_RECIP_MASK;
    const bool in = (r->bmRequestType & USB_REQ_DIR_IN) != 0U;
    switch (r->bRequest) {
    case REQ_GET_STATUS: {
        if (!in) return false;
        usb_ctrl_buf[0] = 0;             /* bus powered, no remote wakeup, not halted */
        usb_ctrl_buf[1] = 0;
        if (recipient == REQ_RECIP_INTERFACE) {
            if (!usb_configuration || r->wIndex > 1U) return false;
        } else if (recipient == REQ_RECIP_ENDPOINT) {
            const int halted = usbd_halted(r->wIndex);
            if (halted < 0) return false;
            usb_ctrl_buf[0] = (uint8_t)halted;
        } else if (recipient != REQ_RECIP_DEVICE) {
            return false;
        }
        usbd_reply(usb_ctrl_buf, 2);
        return true;
    }
    case REQ_CLEAR_FEATURE:
    case REQ_SET_FEATURE:
        /* ENDPOINT_HALT only: no remote wakeup, no test modes */
        if (in || recipient != REQ_RECIP_ENDPOINT || r->wValue != 0U ||
            !usbd_halt(r->wIndex, r->bRequest == REQ_SET_FEATURE)) {
            return false;
        }
        usbd_ack();
        return true;
    case REQ_SET_ADDRESS:
        if (in || recipient != REQ_RECIP_DEVICE || r->wValue > 127U || usb_configuration) return false;
        usbd_set_address((uint8_t)r->wValue);
        return true;
    case REQ_GET_DESCRIPTOR:
        return in && get_descriptor();
    case REQ_GET_CONFIGURATION:
        if (!in || recipient != REQ_RECIP_DEVICE) return false;
        usb_ctrl_buf[0] = usb_configuration;
        usbd_reply(usb_ctrl_buf, 1);
        return true;
    case REQ_SET_CONFIGURATION:
        if (in || recipient != REQ_RECIP_DEVICE || r->wValue > 1U) return false;
        usb_configuration = (uint8_t)r->wValue;
        usbd_configure(usb_configuration);
        usbd_ack();
        return true;
    case REQ_GET_INTERFACE:
        if (!in || recipient != REQ_RECIP_INTERFACE || !usb_configuration || r->wIndex > 1U) return false;
        usb_ctrl_buf[0] = 0;
        usbd_reply(usb_ctrl_buf, 1);
        return true;
    case REQ_SET_INTERFACE:
        if (in || recipient != REQ_RECIP_INTERFACE || !usb_configuration || r->wIndex > 1U || r->wValue != 0U) {
            return false;
        }
        /* Even the current alternate setting resets its endpoints' halts and data toggles
         * (USB 2.0 9.1.1.5, 9.4.5). */
        if (r->wIndex == 0U) {
            (void)usbd_halt(USB_EP_NOTIFY_ADDR, false);
        } else {
            (void)usbd_halt(USB_EP_IN_ADDR, false);
            (void)usbd_halt(USB_EP_OUT_ADDR, false);
        }
        usbd_ack();
        return true;
    default:
        return false;
    }
}

static bool cdc_request(void)
{
    const usb_setup_t* r = &usb_req;
    const bool in = (r->bmRequestType & USB_REQ_DIR_IN) != 0U;
    if ((r->bmRequestType & REQ_RECIP_MASK) != REQ_RECIP_INTERFACE || r->wIndex != 0U || !usb_configuration) {
        return false;
    }
    switch (r->bRequest) {
    case CDC_SET_LINE_CODING:
        return !in && r->wLength == sizeof line_coding && usbd_receive(sizeof line_coding);
    case CDC_GET_LINE_CODING:
        if (!in) return false;
        usbd_reply(line_coding, sizeof line_coding);
        return true;
    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
        if (in) return false;
        usbd_ack();                      /* DTR, RTS and breaks mean nothing to the protocol */
        return true;
    default:
        return false;
    }
}

bool usb_setup(void)
{
    switch (usb_req.bmRequestType & REQ_TYPE_MASK) {
    case REQ_TYPE_STANDARD: return standard_request();
    case REQ_TYPE_CLASS: return cdc_request();
    default: return false;
    }
}

bool usb_setup_out(void)
{
    if ((usb_req.bmRequestType & REQ_TYPE_MASK) == REQ_TYPE_CLASS && usb_req.bRequest == CDC_SET_LINE_CODING) {
        memcpy(line_coding, usb_ctrl_buf, sizeof line_coding);
        return true;
    }
    return false;
}

#endif
