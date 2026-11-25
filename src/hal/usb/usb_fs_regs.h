/*
 * The full-speed USB device peripheral with packet memory (STM32F0, F3; RM0091, RM0316): register
 * offsets from its base and bits, for usb_fs.c.  Checked against ST's headers by tests/regcheck.
 */
#ifndef MUBL_USB_FS_REGS_H
#define MUBL_USB_FS_REGS_H

#define USB_EPR(n) (4U * (n))
#define USB_CNTR 0x40U
#define USB_ISTR 0x44U
#define USB_DADDR 0x4CU
#define USB_BTABLE 0x50U
#define USB_BCDR 0x58U             /* F0 (and F303xD/E-class parts with the internal pull-up) */

#define USB_CNTR_FRES 0x0001U
#define USB_DADDR_EF 0x0080U
#define USB_BCDR_DPPU 0x8000U

/* EPnR (RM0091, RM0316) */
#define EPR_CTR_RX 0x8000U         /* rc_w0 */
#define EPR_DTOG_RX 0x4000U        /* toggle */
#define EPR_STAT_RX 0x3000U        /* toggle */
#define EPR_SETUP 0x0800U          /* read-only */
#define EPR_TYPE 0x0600U           /* rw */
#define EPR_KIND 0x0100U           /* rw */
#define EPR_CTR_TX 0x0080U         /* rc_w0 */
#define EPR_DTOG_TX 0x0040U        /* toggle */
#define EPR_STAT_TX 0x0030U        /* toggle */
#define EPR_EA 0x000FU             /* rw */
#define EPR_RW (EPR_TYPE | EPR_KIND | EPR_EA)
#define EPR_CTR (EPR_CTR_RX | EPR_CTR_TX)

#define EPR_TYPE_BULK 0x0000U
#define EPR_TYPE_CONTROL 0x0200U
#define EPR_TYPE_INTERRUPT 0x0600U

#define RX_DIS 0x0000U
#define RX_STALL 0x1000U
#define RX_NAK 0x2000U
#define RX_VALID 0x3000U
#define TX_DIS 0x0000U
#define TX_STALL 0x0010U
#define TX_NAK 0x0020U
#define TX_VALID 0x0030U

#define ISTR_CTR 0x8000U
#define ISTR_RESET 0x0400U
#define ISTR_SOF 0x0200U
#define ISTR_EP_ID 0x000FU

#endif
