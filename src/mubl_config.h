/*
 * µBL configuration: defaults for everything a board.h may set, and the values derived from them.
 * Preprocessor definitions only (the linker script is generated from this file).
 *
 * A board.h sets at least MUBL_FAMILY, the memory map (MUBL_BL_SIZE, MUBL_APP_SIZE, MUBL_RAM_BASE,
 * MUBL_RAM_SIZE), its links and their pins.  Everything else has a default here.
 */
#ifndef MUBL_CONFIG_H
#define MUBL_CONFIG_H

#include "mubl_defs.h"

#ifndef MUBL_FAMILY
#error "board.h must define MUBL_FAMILY"
#endif

/* ---- Identity ----------------------------------------------------------------------------- */

#ifndef MUBL_BOARD_NAME
#define MUBL_BOARD_NAME "org.sfwa.mubl"           /* DroneCAN GetNodeInfo name */
#endif
#ifndef MUBL_BOARD_ID
#define MUBL_BOARD_ID 0                           /* APJ board ID: serial GET_DEVICE, AP descriptor check */
#endif
#ifndef MUBL_BOARD_REV
#define MUBL_BOARD_REV 0
#endif
#ifndef MUBL_HW_VERSION_MAJOR                     /* DroneCAN HardwareVersion, AP_Periph convention */
#define MUBL_HW_VERSION_MAJOR ((MUBL_BOARD_ID >> 8) & 0xFF)
#endif
#ifndef MUBL_HW_VERSION_MINOR
#define MUBL_HW_VERSION_MINOR (MUBL_BOARD_ID & 0xFF)
#endif
#ifndef MUBL_VERSION_MAJOR
#define MUBL_VERSION_MAJOR 0
#endif
#ifndef MUBL_VERSION_MINOR
#define MUBL_VERSION_MINOR 1
#endif

/* ---- Memory map --------------------------------------------------------------------------- */

#ifndef MUBL_FLASH_BASE
#define MUBL_FLASH_BASE 0x08000000
#endif
#if !defined(MUBL_BL_SIZE) || !defined(MUBL_APP_SIZE) || !defined(MUBL_RAM_BASE) || !defined(MUBL_RAM_SIZE)
#error "board.h must define MUBL_BL_SIZE, MUBL_APP_SIZE, MUBL_RAM_BASE and MUBL_RAM_SIZE"
#endif
#ifndef MUBL_APP_BASE
#define MUBL_APP_BASE (MUBL_FLASH_BASE + MUBL_BL_SIZE)
#endif

/* Flash programming unit and ECC.  The update engine programs whole granules and holds back the
 * first MUBL_HOLD_SIZE bytes of the image (at least the initial SP and reset vector) until the
 * rest is written and verified. */
#ifndef MUBL_FLASH_GRANULE
#if MUBL_FAMILY == MUBL_FAMILY_F0 || MUBL_FAMILY == MUBL_FAMILY_F3
#define MUBL_FLASH_GRANULE 2
#elif MUBL_FAMILY == MUBL_FAMILY_F4
#define MUBL_FLASH_GRANULE 4
#elif MUBL_FAMILY == MUBL_FAMILY_H7
#define MUBL_FLASH_GRANULE 32
#else
#error "MUBL_FLASH_GRANULE is not known for this family"
#endif
#endif
#ifndef MUBL_FLASH_ECC
#define MUBL_FLASH_ECC (MUBL_FAMILY == MUBL_FAMILY_H7)   /* reads can raise uncorrectable-error faults */
#endif
/* CRC-32 (image checks, GET_CRC, SRXL2 FINISH, handoff): in the CRC unit on F3 and H7
 * (MUBL_HW_CRC32; F4's unit cannot do the reflected CRC-32; F0's can, but the driver is larger than
 * a bitwise loop), otherwise in software.  Software CRCs (CRC-32 without the CRC unit, CRC-64 for
 * legacy images and the PX4 handoff) use 16-entry tables (64 B, 128 B) unless MUBL_CRC32_TABLE /
 * MUBL_CRC64_TABLE are 0: on F0 both run bit by bit, smaller and slower. */
#ifndef MUBL_HW_CRC32
#define MUBL_HW_CRC32 (MUBL_FAMILY == MUBL_FAMILY_F3 || MUBL_FAMILY == MUBL_FAMILY_H7)
#endif
#ifndef MUBL_CRC32_TABLE
#define MUBL_CRC32_TABLE (MUBL_FAMILY != MUBL_FAMILY_F0)
#endif
#ifndef MUBL_CRC64_TABLE
#define MUBL_CRC64_TABLE (MUBL_FAMILY != MUBL_FAMILY_F0)
#endif

#if MUBL_FLASH_GRANULE > 8
#define MUBL_HOLD_SIZE MUBL_FLASH_GRANULE
#else
#define MUBL_HOLD_SIZE 8
#endif

/* RAM kept out of the bootloader's own use, at the start of RAM, in this order:
 *   - on Cortex-M0 (no VTOR), room for the application's vector table, which the bootloader copies
 *     there and maps at address 0 before jumping;
 *   - with MUBL_COMPAT_AP_PERIPH, AP_Periph's app_bootloader_comms structure;
 *   - the µBL handoff block (app/mubl_app.h).
 * The application must not place data here either (see app/README.md). */
#ifndef MUBL_VECTOR_RESERVE
#if MUBL_FAMILY == MUBL_FAMILY_F0
#define MUBL_VECTOR_RESERVE 0xC0                  /* 48 vectors: the largest F0 table (F09x) */
#else
#define MUBL_VECTOR_RESERVE 0
#endif
#endif
#ifndef MUBL_COMPAT_AP_PERIPH
#define MUBL_COMPAT_AP_PERIPH 0                   /* read AP_Periph's app_bootloader_comms */
#endif
#ifndef MUBL_AP_COMMS_ADDR
#define MUBL_AP_COMMS_ADDR (MUBL_RAM_BASE + MUBL_VECTOR_RESERVE)
#endif
#ifndef MUBL_HANDOFF_SIZE
#define MUBL_HANDOFF_SIZE 256
#endif
#ifndef MUBL_HANDOFF_ADDR
#define MUBL_HANDOFF_ADDR (MUBL_RAM_BASE + MUBL_VECTOR_RESERVE + (MUBL_COMPAT_AP_PERIPH ? 256 : 0))
#endif
#ifndef MUBL_RAM_RESERVED
#define MUBL_RAM_RESERVED (MUBL_HANDOFF_ADDR - MUBL_RAM_BASE + MUBL_HANDOFF_SIZE)
#endif
/* Other RAM an application may put its initial stack in, as {start, end} pairs (end exclusive):
 * CCM on F3/F4, AXI SRAM and SRAM1-4 on H7.  The bootloader's own RAM (MUBL_RAM_BASE/SIZE) is
 * always allowed.  An image whose initial SP is anywhere else is not booted. */
/* #define MUBL_STACK_RAM {0x10000000, 0x10010000} */

#ifndef MUBL_MIN_STACK
#define MUBL_MIN_STACK 1024                       /* checked by the linker */
#endif
#ifndef MUBL_COMPAT_PX4_CAN
#define MUBL_COMPAT_PX4_CAN 0                     /* PX4/thiemar bxCAN filter-register handoff */
#endif

/* ---- Boot policy -------------------------------------------------------------------------- */

#ifndef MUBL_BOOT_DELAY_MS
#define MUBL_BOOT_DELAY_MS 1000       /* with a valid application: time to wait for a host first */
#endif
#ifndef MUBL_DETECT_EXTEND_MS
#define MUBL_DETECT_EXTEND_MS 0       /* added to the boot delay once when the first link is detected */
#endif
#ifndef MUBL_LINK_EXCLUSIVE
#define MUBL_LINK_EXCLUSIVE 0         /* 1: a detected link stops every link of lower priority */
#endif
#ifndef MUBL_MAX_BOOT_ATTEMPTS
#define MUBL_MAX_BOOT_ATTEMPTS 0      /* stay after this many boots the application did not confirm */
#endif
#ifndef MUBL_RAW_IMAGES
#define MUBL_RAW_IMAGES 0             /* 1: also boot images with no descriptor (stock ArduPilot, Betaflight) */
#endif
#ifndef MUBL_IMAGE_FORMATS
#define MUBL_IMAGE_FORMATS (MUBL_IMG_LEGACY | MUBL_IMG_AP | (MUBL_RAW_IMAGES ? MUBL_IMG_RAW : 0))
#endif
#if MUBL_RAW_IMAGES && !(MUBL_IMAGE_FORMATS & MUBL_IMG_RAW)
#error "MUBL_RAW_IMAGES needs MUBL_IMG_RAW in MUBL_IMAGE_FORMATS"
#endif
#ifndef MUBL_CHECK_BOARD_ID
#define MUBL_CHECK_BOARD_ID 1         /* AP descriptor: board_id must be MUBL_BOARD_ID (or _ALT) */
#endif
#ifndef MUBL_DESC_SCAN_LIMIT
#define MUBL_DESC_SCAN_LIMIT MUBL_APP_SIZE   /* bytes of the image searched for a descriptor */
#endif
#ifndef MUBL_WATCHDOG
#define MUBL_WATCHDOG 0               /* 1: start the IWDG (it can then never be stopped) */
#endif
#ifndef MUBL_WATCHDOG_MS
#define MUBL_WATCHDOG_MS 8000         /* IWDG period when started here or by the hardware-watchdog option */
#endif

/* ---- Board pins (optional) --------------------------------------------------------------- */

/* MUBL_HOLD_PIN: an input that keeps the bootloader running; MUBL_LED_PIN: on in the bootloader,
 * blinking while writing.  MUBL_CAN_STANDBY_PIN: a transceiver standby input, driven to the
 * level that is not MUBL_CAN_STANDBY_LEVEL.  MUBL_OUTPUT_PINS: outputs held at a level while the
 * bootloader runs (chip selects, power switches), {pin, level} pairs:
 *   #define MUBL_OUTPUT_PINS {MUBL_PIN('B', 12, 0), 1}, {MUBL_PIN('C', 13, 0), 0}
 * Like every pin the bootloader uses, their ports are reset before the application starts. */
#ifndef MUBL_HOLD_LEVEL
#define MUBL_HOLD_LEVEL 0             /* 0: active low (pulled up), 1: active high (pulled down) */
#endif
#ifndef MUBL_LED_ON
#define MUBL_LED_ON 1
#endif
#ifndef MUBL_CAN_STANDBY_LEVEL
#define MUBL_CAN_STANDBY_LEVEL 1
#endif

/* ---- Links -------------------------------------------------------------------------------- */

#ifndef MUBL_DRONECAN
#define MUBL_DRONECAN 0
#endif
#ifndef MUBL_SERIAL
#define MUBL_SERIAL 0
#endif
#ifndef MUBL_SRXL2
#define MUBL_SRXL2 0
#endif
#ifndef MUBL_USB
#define MUBL_USB 0                    /* the serial protocol over USB CDC-ACM (MUBL_LINK_USB) */
#endif
#if !MUBL_DRONECAN && !MUBL_SERIAL && !MUBL_SRXL2 && !MUBL_USB
#error "board.h enables no link (MUBL_DRONECAN, MUBL_SERIAL, MUBL_SRXL2, MUBL_USB)"
#endif
#ifndef MUBL_LINK_PRIORITY
#define MUBL_LINK_PRIORITY MUBL_LINK_USB, MUBL_LINK_DRONECAN, MUBL_LINK_SRXL2, MUBL_LINK_SERIAL
#endif

/* DroneCAN */
#ifndef MUBL_CAN_FD
#define MUBL_CAN_FD 0                 /* 1: CAN FD frames (FDCAN only); the bus must be CAN FD */
#endif
#ifndef MUBL_CAN_INSTANCE
#define MUBL_CAN_INSTANCE 1
#endif
#ifndef MUBL_CAN_BITRATE
#define MUBL_CAN_BITRATE 0            /* 0: detect 1M/500k/250k/125k; otherwise fixed */
#endif
#ifndef MUBL_CAN_AUTOBAUD_DWELL_MS
#define MUBL_CAN_AUTOBAUD_DWELL_MS 1100   /* longest wait at a bitrate with no traffic or errors */
#endif
#ifndef MUBL_CAN_NODE_ID
#define MUBL_CAN_NODE_ID 0            /* 0: dynamic allocation; otherwise static */
#endif
#ifndef MUBL_CAN_PREFERRED_NODE_ID
#define MUBL_CAN_PREFERRED_NODE_ID 0  /* asked for in dynamic allocation */
#endif
#ifndef MUBL_CAN_TX_TIMEOUT_MS
#define MUBL_CAN_TX_TIMEOUT_MS 50     /* a frame waits this long for a mailbox, then is dropped */
#endif
#ifndef MUBL_DC_READ_TIMEOUT_MS
#define MUBL_DC_READ_TIMEOUT_MS 1000  /* file.Read response */
#endif
#ifndef MUBL_DC_READ_RETRIES
#define MUBL_DC_READ_RETRIES 5
#endif
#ifndef MUBL_DC_HOLD_ON_GETNODEINFO
#define MUBL_DC_HOLD_ON_GETNODEINFO 0 /* 1: a GetNodeInfo request keeps the bootloader running */
#endif
#ifndef MUBL_DC_LOG
#define MUBL_DC_LOG 1                 /* uavcan.protocol.debug.LogMessage on update start/end */
#endif

/* UART ports (0 and 1).  MUBL_UARTn_INSTANCE is the USART number; a port with no RX pin is
 * single-wire half-duplex on its TX pin (required for SRXL2). */
#ifndef MUBL_UART0_INSTANCE
#define MUBL_UART0_INSTANCE 0
#endif
#ifndef MUBL_UART1_INSTANCE
#define MUBL_UART1_INSTANCE 0
#endif
#ifndef MUBL_UART_SLICE_MS
#define MUBL_UART_SLICE_MS 250        /* listening time per baud rate on a port shared at two rates */
#endif

/* Serial (PX4/ArduPilot bootloader protocol) */
#ifndef MUBL_SERIAL_PORT
#define MUBL_SERIAL_PORT 0
#endif
#ifndef MUBL_SERIAL_BAUD
#define MUBL_SERIAL_BAUD 115200
#endif
#ifndef MUBL_SERIAL_CHIP_DES
#define MUBL_SERIAL_CHIP_DES "STM32,?"   /* GET_CHIP_DES: "family,revision" */
#endif

/* SRXL2 */
#ifndef MUBL_SRXL2_PORT
#define MUBL_SRXL2_PORT 0
#endif
#ifndef MUBL_SRXL2_DEVICE_ID
#define MUBL_SRXL2_DEVICE_ID 0x40     /* 0x40: ESC; 0x60: servo; 0x30: flight controller (spec 7.1.1) */
#endif
#ifndef MUBL_SRXL2_400K
#define MUBL_SRXL2_400K 1             /* offer 400000 baud in the handshake */
#endif
#ifndef MUBL_SRXL2_DFU_TYPE
#define MUBL_SRXL2_DFU_TYPE 0xB0      /* packet type of the µBL update extension */
#endif
#ifndef MUBL_SRXL2_REPLY_DELAY_US
#define MUBL_SRXL2_REPLY_DELAY_US 200 /* idle before replying: at least 2 characters (spec 6.2 f) */
#endif

/* USB (the serial protocol over CDC-ACM).  The device attaches when the link starts.  A host
 * driving the bus (start-of-frame packets) within the boot delay keeps the bootloader running
 * until the host goes away (no frames for MUBL_USB_GONE_MS: unplugged or suspended), a BOOT
 * command, or a reset.  Pins: F0's USB pins need no set-up; elsewhere set MUBL_USB_DM_PIN and
 * MUBL_USB_DP_PIN (with their alternate function), and MUBL_USB_PULLUP_PIN where D+ is pulled up
 * by an external switch (driven to MUBL_USB_PULLUP_LEVEL to attach). */
#ifndef MUBL_USB_OTG_CORE
#define MUBL_USB_OTG_CORE (MUBL_FAMILY == MUBL_FAMILY_F4 || MUBL_FAMILY == MUBL_FAMILY_H7)  /* else the F0/F3 one */
#endif
#ifndef MUBL_USB_STAY
#define MUBL_USB_STAY 1               /* 0: USB is like a UART: boot after the delay unless used */
#endif
#ifndef MUBL_USB_GONE_MS
#define MUBL_USB_GONE_MS 500
#endif
#ifndef MUBL_USB_VID
#define MUBL_USB_VID 0x0483           /* ST's virtual COM port IDs, as µCAN */
#endif
#ifndef MUBL_USB_PID
#define MUBL_USB_PID 0x5740
#endif
#ifndef MUBL_USB_BCD_DEVICE
#define MUBL_USB_BCD_DEVICE ((MUBL_VERSION_MAJOR << 8) | MUBL_VERSION_MINOR)
#endif
#ifndef MUBL_USB_MANUFACTURER
#define MUBL_USB_MANUFACTURER "muBL"
#endif
#ifndef MUBL_USB_PRODUCT
#define MUBL_USB_PRODUCT MUBL_BOARD_NAME
#endif
#ifndef MUBL_USB_MAX_POWER_MA
#define MUBL_USB_MAX_POWER_MA 100
#endif
#ifndef MUBL_USB_TX_SIZE
#define MUBL_USB_TX_SIZE 64           /* transmit ring: a power of two; replies are at most 22 bytes */
#endif
#ifndef MUBL_USB_FLUSH_MS
#define MUBL_USB_FLUSH_MS 20          /* before detaching: time for the host to collect the last reply */
#endif
#ifndef MUBL_USB_DETACH_MS
#define MUBL_USB_DETACH_MS 20         /* detached this long before the application starts */
#endif
#ifndef MUBL_USB_PULLUP_LEVEL
#define MUBL_USB_PULLUP_LEVEL 1
#endif

#endif /* MUBL_CONFIG_H */
