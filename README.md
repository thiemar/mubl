# µBL

A small (< 8 KiB for any one interface, < 16 KiB for all four), freestanding C bootloader for STM32 with
firmware update over **USB**, **serial**, **DroneCAN**, and **Spektrum SRXL2**.  STM32F0, F3, F4, G4 and H7
are supported as shipped; other parts are added with a board file and, where the peripherals differ,
a family header.  It descends from [thiemar/bootloader](https://github.com/thiemar/bootloader) and uses
[sfwa-dronecan](https://github.com/sfwa/dronecan)'s bootloader profile for the standard DroneCAN wire format.

Serial and USB use the **PX4/ArduPilot serial bootloader protocol** so existing tools can be used without modification.

Status: everything above the register level is tested on a host simulation (290 tests, including
ArduPilot's `uploader.py` over a UART and over simulated USB, sfwa-dronecan's `FirmwareUpdater`,
Spektrum's reference SRXL2 code, µCAN's USB driver tests, power loss injected at every flash
operation of an update, and sanitizer-checked fuzzing of all four links).  The STM32 code builds for every example board and its register definitions are
checked against ST's CMSIS headers.  **On hardware, only USB on two H7 boards has run so far**: the
STM32H735G-DK (`h735g-dk`) and a TBS Lucid H7 (`tbs-lucid-h7`), which boots stock ArduCopter (see
[Hardware tests](#hardware-tests)); DroneCAN, the UART links and the F0/F3/F4/G4 parts have not.

## Contents

- [How it boots](#how-it-boots)
- [Links: detection and priority](#links-detection-and-priority)
- [Updating safely](#updating-safely)
- [DroneCAN](#dronecan) · [Serial](#serial) · [USB](#usb) · [SRXL2](#srxl2)
- [Images](#images) · [Handoff](#handoff)
- [Boards and configuration](#boards-and-configuration) · [Adding a part](#adding-a-part)
- [Building](#building) · [Bring-up](#bring-up-on-a-new-board) · [Size](#size) · [Testing](#testing) · [Limitations](#limitations)

## How it boots

After reset µBL reads the handoff the application may have left, checks the application, starts
its links and polls them.  It starts the application `MUBL_BOOT_DELAY_MS` (default 1 s) after reset unless:

| Stays because | |
| --- | --- |
| no valid application | first word erased; initial SP not in this board's RAM or reset vector not Thumb code in the application region; or a descriptor that fails (size, CRC, board ID, reset vector outside the image it describes) |
| the application asked | handoff request: native, AP_Periph's, or PX4's (below) |
| hold pin | `MUBL_HOLD_PIN` asserted at reset |
| a host started using a link | serial `GET_SYNC` + identify, any SRXL2 update packet, DroneCAN BeginFirmwareUpdate (and GetNodeInfo with `MUBL_DC_HOLD_ON_GETNODEINFO`) |
| boot loop | the application did not confirm `MUBL_MAX_BOOT_ATTEMPTS` boots in a row |
| a USB host is connected | start-of-frame packets within the boot delay (`MUBL_USB_STAY`, on by default); until the host goes away (`MUBL_USB_GONE_MS` without frames: unplugged or suspended), `BOOT` or a reset |

Traffic alone never holds the bootloader: on an aircraft the CAN bus and the SRXL2 receiver are
always talking, and a valid application must still start on time.  An update in progress always
holds it.

Leaving: links stop, peripherals and the GPIO ports used are reset, clocks return to the reset
configuration, the handoff block is written, then VTOR (or on Cortex-M0, a copy of the vectors to
SRAM mapped at address 0), MSP and a jump.  See [app/README.md](app/README.md) for what the
application can rely on.

## Links: detection and priority

A board lists its links in `MUBL_LINK_PRIORITY`, highest first.  At start every link listens at
once.  The serial protocol says nothing until a host has synced; SRXL2 announces itself only as the
specification requires; DroneCAN transmits nothing until it knows the bitrate (configured, handed
off, or detected from traffic it has heard).

| Link | Detected when |
| --- | --- |
| DroneCAN | a frame is received.  Without a configured or handed-off bitrate the controller listens silently (no ACKs) at 1M, 500k, 250k, 125k in turn, moving on at the first protocol error or after `MUBL_CAN_AUTOBAUD_DWELL_MS` of silence |
| serial | a well-formed `GET_SYNC` arrives |
| SRXL2 | a packet with a valid CRC arrives |
| USB | the host sends start-of-frame packets (it has reset the bus after the device attached) |

Priority decides:

- **who speaks first on a shared UART.**  Serial and SRXL2 may share one port (typically the
  single-wire SRXL2 line of an ESC or servo, also used for bench flashing).  Both parsers see every
  byte until one decodes a valid frame and claims the port.  Before that only the higher-priority
  protocol may transmit: with SRXL2 first the device announces itself; with serial first it stays
  silent until polled.  If the two use different baud rates the port alternates between them every
  `MUBL_UART_SLICE_MS` (250 ms), in priority order.
- **which links survive detection** with `MUBL_LINK_EXCLUSIVE`: the first link to find a peer stops
  every link of lower priority (higher ones keep listening).  Without it every link stays up and
  the first to start an update owns it; the others answer "busy" until it ends.
- **start-up order** and poll order.

`MUBL_DETECT_EXTEND_MS` optionally lengthens the boot delay once when a link is first detected.
An application can also name one link in its handoff request, which then runs alone with the
parameters given (bitrate, node ID, baud rate, SRXL2 device ID).

## Updating safely

All links drive one update engine (`src/update.c`):

- **Sequential writes, erase on demand.**  Each sector is erased when the first byte for it arrives
  (blank sectors are skipped), so links keep running between erases and small images do not pay for
  a whole-region erase.  The first sector is erased when the update begins, which makes the old
  application unbootable immediately.
- **First block last.**  The first `max(8, programming unit)` bytes (initial SP and reset vector)
  are held in RAM.  When the image is complete, sectors after it that are not blank are erased, the
  image is validated with the held bytes substituted, and only then are they programmed, last unit
  first, so the initial SP is the last thing written.  Every programmed unit is read back.
- **Programming granule** per family: half-words (F0/F3), words (F4), 64-bit ECC double words (G4), 256-bit ECC flash words (H7).
- **ECC.**  On G4 and H7 an interrupted erase or program can leave flash that faults when read (NMI on G4, bus fault on H7).  Image
  checks and CRCs run under `hal_guarded`, which turns that fault into "invalid image" instead of
  a reset loop.

The simulation cuts power at each flash operation of an update over a previous image, leaving the
operation half done (random bits, ECC damage): every one of 12,936 operations on the F0 geometry,
11,255 on F4 and 1,408 on H7 (`MUBL_EXHAUSTIVE=1`; the default run takes 400 per geometry).  In
every case the bootloader afterwards either stays or starts a byte-identical complete image, never
reads damaged flash outside `hal_guarded`, and a fresh update succeeds.

## DroneCAN

Built on sfwa-dronecan's bootloader profile (`dc_tiny`: one reassembly at a time, no queue, no C
library), generated into `src/dronecan/{classic,fd}` by `tools/gen_dronecan.sh`.

- **Node ID**: from the handoff, else `MUBL_CAN_NODE_ID`, else dynamic allocation following rules
  A–E of `uavcan.protocol.dynamic_node_id.Allocation` (randomised request period and follow-up
  delay, preferred ID `MUBL_CAN_PREFERRED_NODE_ID`).
- **Services**: NodeStatus at 1 Hz (MAINTENANCE, SOFTWARE_UPDATE while updating; health WARNING
  without an application, ERROR after a failed update; vendor status = application check result |
  stay reason << 8), GetNodeInfo (version, VCS commit and image CRC from the descriptor; hardware
  version from the board ID; 16-byte unique ID), RestartNode, BeginFirmwareUpdate.
- **Update**: `uavcan.protocol.file.Read`, 256 bytes at a time, one request outstanding, 1 s timeout,
  5 retries; a short read ends the file.  The node boots the image if it validates, otherwise stays
  with health ERROR.  Optional LogMessage on start, success and failure (`MUBL_DC_LOG`).
- **CAN FD** on G4 and H7 (`MUBL_CAN_FD`, optional bit-rate switching with `MUBL_CAN_FD_DATA_BITRATE`, with
  transmitter delay compensation).  FDCAN recovers from bus-off itself; bxCAN uses automatic bus-off
  recovery.
- One interface; hardware filters pass only service frames addressed to the node once it has an ID.

Interoperability: tested against sfwa-dronecan's pydronecan-compatible `CentralizedServer`,
`NodeMonitor`, `FileServer` and `FirmwareUpdater` on its virtual bus.

## Serial

The PX4/ArduPilot bootloader protocol, revision 5, as `uploader.py`, QGroundControl and Mission
Planner use it: `GET_SYNC`, `GET_DEVICE` (BL_REV 5, BOARD_ID, BOARD_REV, FW_SIZE, VEC_AREA,
EXTF_SIZE 0), `CHIP_ERASE`, `PROG_MULTI` (≤ 252 bytes), `GET_CRC`, `GET_OTP`, `GET_SN`, `GET_CHIP`,
`GET_CHIP_DES`, `GET_VERSION`, `SET_BAUD`, `BOOT`.  Erase and program need sync and identify first.
`GET_CRC` covers the region as it will be after `BOOT`; `BOOT` validates the image and answers
FAILED if it does not pass.  ArduPilot's `uploader.py` runs against it unmodified (on a dedicated
port and on a port shared with SRXL2).

## USB

The serial protocol over USB CDC-ACM (`MUBL_USB`, link `MUBL_LINK_USB`): a second instance of the
same protocol, so `uploader.py`, QGroundControl and Mission Planner work through the USB port as
through a UART.  `SET_BAUD` is acknowledged and ignored.  Only one link owns an update at a time.

- **Attach and stay.**  The device attaches when the link starts.  A host that enumerates it within
  the boot delay (hosts reset the bus about 100 ms after attach) keeps the bootloader running while
  it is there; on a charger nothing happens and the application starts on time.  There is no VBUS
  sensing: a host that suspends the bus counts as gone.
- **Leaving.**  `BOOT` is answered, the reply flushed, then the device detaches for
  `MUBL_USB_DETACH_MS` before the application starts, so the host sees a new device; the application
  enumerates its own USB afresh.  An application asks for the bootloader over USB with the handoff
  (link 4), then resets; only USB then runs.
- **Drivers.**  `src/hal/usb/usb_cdc.c` holds the descriptors and requests; `usb_fs.c` drives the
  F0/F3/G4 USB peripheral (ported from µCAN's driver), `usb_otg.c` the OTG core of F4 and H7.  Both are
  polled; the USB interrupt is not used.  Received data is taken only as fast as the protocol
  consumes it, so the host is held off (NAK) while flash is busy.
- **Clocks.**  F0: the crystal's PLL, or HSI48 trimmed by the CRS against the host's frames.  F3, F4:
  the crystal only (USB stays off the bus if it fails).  G4: HSI48 with the CRS.  H7: HSI48 with the CRS, and VDD33USB.
- **IDs.**  `MUBL_USB_VID`/`PID` default to ST's virtual COM port (0483:5740, as µCAN); the
  product string is the board name, the serial number the unique ID.

| Family | Peripheral | Board settings |
| --- | --- | --- |
| F0 (F04x/F07x/F09x) | USB, internal D+ pull-up | none (PA11/PA12 are dedicated) |
| F3 (F302/F303) | USB, no internal pull-up | `MUBL_USB_DM_PIN`/`DP_PIN` (AF14); `MUBL_USB_PULLUP_PIN` for a switched pull-up, otherwise D+ is driven low to detach; `MUBL_F3_USB_PMA_16BIT` on xD/xE |
| F4 | OTG_FS | `MUBL_USB_DM_PIN`/`DP_PIN` (AF10); `MUBL_F4_OTG_VBDEN` on F446/F412-class cores; a system clock whose VCO is a multiple of 48 MHz |
| G4 | USB, internal D+ pull-up | none (PA11/PA12 are dedicated analog USB pads); HSI48 trimmed by the CRS against USB SOF |
| H7 | OTG_HS or OTG_FS, embedded FS PHY | `MUBL_USB_OTG_INSTANCE` (1: OTG_HS, 2: OTG_FS), its pins (AF10 on PA11/PA12, AF12 on PB14/PB15) |

## SRXL2

As a slave device per the Spektrum SRXL2 specification (Rev K): 115200 baud single-wire
half-duplex, listen 50 ms after start, announce with a Handshake every 50 ms for 200 ms if the unit
ID nibble is 0, answer Handshakes addressed to the device, switch to 400000 baud on the master's
broadcast Handshake, return to 115200 and handshake again after 50 ms of master silence.  Replies
wait `MUBL_SRXL2_REPLY_DELAY_US` (two character times).  The device ID is `MUBL_SRXL2_DEVICE_ID` or
from the handoff.

The specification has no firmware-update packet, so µBL adds one (packet type
`MUBL_SRXL2_DFU_TYPE`, 0xB0), master-driven like everything on the bus:

```
master -> device:  A6 B0 len  dest src op seq  data...            CRC16 (XMODEM, big endian)
device -> master:  A6 B0 len  master device op|0x80 seq  status data...  CRC16
op 1 INFO                  -> protocol, app status/format, board ID, region size, versions, VCS, size, UID
op 2 BEGIN  size:4         erases the first sector (without the size: bad request, nothing erased)
op 3 WRITE  offset:4 data  (1-64 bytes) -> next offset:4
op 4 FINISH size:4 crc:4   zlib CRC-32 of the image; validate, make bootable -> app status
op 5 BOOT
```

A request repeating the previous `src, op, seq` gets the previous reply without being executed
again.  `tools/mubl_srxl2.py` is the host side (bus master over a USB-serial adapter wired for
half duplex).

The handshake is checked differentially against Spektrum's reference implementation (which is
public only for slaves): both receive the same master traffic in lockstep and must answer it
byte-for-byte alike.  They differ only where the reference's behaviour depends on how often its host
calls `srxlRun`: it keeps announcing until polled (µBL stops after 200 ms, as the spec allows), and
it answers a master's first poll only on the second.

## Images

`tools/mubl_image.py stamp` fills a descriptor's size and CRCs; `apj` makes a `.apj` for
`uploader.py`.  `MUBL_IMAGE_FORMATS` chooses what boots:

| Format | Signature | Check |
| --- | --- | --- |
| `MUBL_IMG_AP` | `40 a2 e4 f1 64 68 91 06` | ArduPilot/PX4 unsigned: CRC-32 up to the signature and from `version_major` to the end, `board_id == MUBL_BOARD_ID` |
| `MUBL_IMG_LEGACY` | `APDesc00` | thiemar/early PX4: CRC-64-WE of the whole image with the CRC field zero |
| `MUBL_IMG_RAW` | none | initial SP and reset vector only |

Every format also needs the initial SP inside RAM the board has: its own `MUBL_RAM_BASE`/`SIZE` or
a bank listed in `MUBL_STACK_RAM` (CCM, AXI SRAM, SRAM1-4; the example boards list theirs).  With
a descriptor the reset vector must also lie inside the image it describes, not just the region.

The default is `MUBL_IMG_AP | MUBL_IMG_LEGACY`.  `MUBL_RAW_IMAGES=1` adds `MUBL_IMG_RAW`, for
applications built without a descriptor: stock ArduPilot vehicle firmware (it carries one only in
AP_Periph and Remote ID builds) and Betaflight.  An image with a descriptor is still judged by it, so
a bad CRC or another board's ID is rejected rather than booted as raw.  A raw image has no CRC: a
corrupted one is not caught, but an interrupted update still never boots, because the first block
(the vectors) is written last.  `tbs-lucid-h7` sets `MUBL_RAW_IMAGES` by default.

CRCs, by the instruction counts of the generated loops (not measured):

| | CRC-32 | CRC-64 (legacy format, PX4 handoff) |
| --- | --- | --- |
| F3, G4, H7 | CRC unit (`MUBL_HW_CRC32`): words fed with each byte's bits reversed, result reversed; about 3 cycles a byte | 16-entry table |
| F4 | software, 16-entry table (its CRC unit cannot do the reflected CRC-32) | 16-entry table |
| F0 | software, bit by bit: about 90 cycles a byte, so about 45 ms for a 24 KB image at 48 MHz | bit by bit |

F0 trades boot time for size: no tables (`MUBL_CRC32_TABLE`, `MUBL_CRC64_TABLE`) and no CRC-unit
driver, which there would cost more than the 50-byte loop it replaces.  `MUBL_HW_CRC32` turns the
driver on for an F0 board that wants the faster check.

The descriptor is found by scanning from offset 8, word aligned; its size is bounded by the
application region before anything is read.  A raw image is protected against interrupted updates
only by the first word being written last: on a part that programs a word or more at a time a power
failure during that one write could, in principle, leave a plausible but wrong stack pointer.  Use a
descriptor where that matters.  Signed images are not supported.

## Handoff

`MUBL_HANDOFF_SIZE` (256) bytes of RAM at `MUBL_HANDOFF_ADDR`, CRC-protected with a magic for each
direction and cleared as soon as it is read, so a later reset cannot replay it.

- **Application → bootloader**: stay; use one link; CAN node ID, bitrate, file server and path
  (DroneCAN then starts reading immediately); UART baud rate; SRXL2 device ID.
- **Bootloader → application**: the link used, CAN node ID and bitrate, baud rate, SRXL2 ID and the
  number of unconfirmed boots.  The application confirms a good boot by clearing the magic.
- **AP_Periph** (`MUBL_COMPAT_AP_PERIPH`): `app_bootloader_comms` at the start of RAM is honoured.
- **PX4/thiemar** (`MUBL_COMPAT_PX4_CAN`, bxCAN): the shared structure in CAN1 filter banks 2-4
  (signature, bus speed, node ID, CRC-64) is read at start and written for the application.

`app/mubl_app.h` is the application side.

## Boards and configuration

A board is `boards/<name>/board.h` (C macros) and `board.mk` (`FAMILY := f0|f3|f4|g4|h7`, `CAN_FD := 1`).

| Board | Part | Links | Notes |
| --- | --- | --- | --- |
| `f042-can` | STM32F042K6 | DroneCAN | 8 KiB bootloader |
| `f042-node` | STM32F042K6 | DroneCAN, SRXL2 + serial on one wire | 12 KiB |
| `f042-usb` | STM32F042C6 (µCAN) | USB (DroneCAN with `-DMUBL_DRONECAN=1`) | 8 KiB (13 KiB with DroneCAN); no crystal |
| `f303-node` | STM32F303CC | DroneCAN, serial, SRXL2 | PX4 handoff |
| `f303-usb` | STM32F303CC | USB, serial, SRXL2 | fixed D+ pull-up; no CAN (shared packet memory) |
| `f405-node` | STM32F405RG | USB, serial, DroneCAN, SRXL2 | sector 0 (16 KiB, 156 B free); AP_Periph and PX4 handoff |
| `matek-mr24-30` | STM32G431KB (Mateksys mR24-30) | USB, serial | 16 KiB bootloader, 108 KiB app, top 4 KiB config kept; PA/LNA/fan held off |
| `g474-node` | STM32G474RE | USB, DroneCAN (CAN FD), serial, SRXL2 | dual-bank 512 KiB flash, 24 MHz crystal |
| `h743-node` | STM32H743ZI (Nucleo) | USB (OTG_FS), DroneCAN (FDCAN1), serial (VCP), SRXL2 | LDO supply, HSE bypass |
| `h735-node` | STM32H735 | USB (OTG_HS), DroneCAN over CAN FD, serial | direct SMPS; check against your board |
| `h735g-dk` | STM32H735IG (ST STM32H735G-DK) | USB (OTG_HS) | direct SMPS; LED1, user button |
| `tbs-lucid-h7` | STM32H743 (TBS Lucid H7) | USB (OTG_FS), DroneCAN (FDCAN1), serial (UART7) | ArduPilot layout and board ID (variants by `MUBL_BOARD_ID`); parameter sectors kept; boots images without a descriptor (`MUBL_RAW_IMAGES`) |

The settings and their defaults are in [src/mubl_config.h](src/mubl_config.h).  The ones every
board sets: `MUBL_FAMILY`, `MUBL_BL_SIZE`, `MUBL_APP_SIZE`, `MUBL_RAM_BASE`, `MUBL_RAM_SIZE`,
`MUBL_HSE_HZ`, the links (`MUBL_DRONECAN`, `MUBL_SERIAL`, `MUBL_SRXL2`, `MUBL_USB`, `MUBL_LINK_PRIORITY`) and
their pins: `MUBL_CAN_RX_PIN`/`TX_PIN`, `MUBL_UARTn_INSTANCE`/`TX_PIN`/`RX_PIN` (no RX pin makes a
single-wire half-duplex port), `MUBL_SERIAL_PORT`, `MUBL_SRXL2_PORT`.  H7 boards must also set
`MUBL_H7_PWR_CR3` to the supply their hardware is wired for.  Boards with RAM besides
`MUBL_RAM_BASE`/`SIZE` list it in `MUBL_STACK_RAM` if an application may put its stack there.  Pins are `MUBL_PIN('A', 11, 4)`:
port, number, alternate function.  Optional pins: `MUBL_LED_PIN`, `MUBL_HOLD_PIN`, and
`MUBL_OUTPUT_PINS`, outputs held at a level while the bootloader runs (chip selects, power
switches: `tbs-lucid-h7` keeps its VTX supplies off, as ArduPilot's bootloader does).

## Adding a part

A part whose peripherals match a supported family (most STM32F0/F3/F4/G4/H74x/H72x/H73x) needs only a
board file: flash and RAM sizes, pins, clock, and for F0 parts with 2 KiB pages `MUBL_FLASH_PAGE`.
A new family needs `src/hal/stm32/family_<x>.h` (base addresses and enable bits), its clock set-up in
`clock.c`, and its flash routine in `flash.c` if the controller differs (G0/L4, like G4, program 64-bit
double words with ECC; H7A3/B0 use 128-bit words and 8 KiB sectors).  bxCAN, FDCAN, USART v1/v2 and
GPIO are shared.  Add the part to `tests/regcheck/run.py` so its registers are checked against ST's
header.  Another vendor needs its own `src/hal/<vendor>` implementing `src/hal/hal.h`.

## Building

```
make BOARD=f303-node          # build/f303-node/mubl.{elf,bin,hex}
make all-boards
make test                     # host simulation tests
make regcheck                 # register definitions against ST CMSIS headers
make sizes                    # flash, RAM and stack per board
```

The Makefile uses the Arm GNU Toolchain in `../toolchains` if present, else `arm-none-eabi-gcc` on
the PATH (`CROSS=` to choose).  Builds use LTO, `-Os`, no C library, and `-Werror`.

## Bring-up on a new board

Only USB on `h735g-dk` and `tbs-lucid-h7` has run on hardware so far, so on other boards check these in order the first time:

1. Flash the bootloader alone: `st-flash write build/<board>/mubl.bin 0x08000000` (or
   `STM32_Programmer_CLI -c port=SWD -w build/<board>/mubl.bin 0x08000000`).  With no application it
   stays; the LED (if `MUBL_LED_PIN`) is on.
2. Clocks: SysTick drives every timeout, so a wrong `MUBL_HSE_HZ` shows up as timing that is off by
   the same ratio (NodeStatus not at 1 Hz, uploader timeouts).  On H7 confirm `MUBL_H7_PWR_CR3`
   against the schematic *before* flashing.
3. Serial: `uploader.py --port <port> --identify` should report protocol 5 and the board ID.  USB:
   plugging in should show a CDC-ACM port with the board name and unique ID within the boot delay
   (`lsusb -v`, System Information), and the same command should work on it.  If it does not
   enumerate, check the 48 MHz clock (F0 without crystal: HSI48 and the CRS; H7: VDD33USB) and, on
   F3, the D+ pull-up.
4. DroneCAN: on a bus with traffic the node detects the bitrate, gets a node ID from an allocator
   and appears in the DroneCAN GUI tool with the board name; then update it from there.
5. SRXL2: `tools/mubl_srxl2.py <port> app.bin` through a half-duplex adapter.
6. Interrupted update: cut power during an update and check the board comes back in the
   bootloader; on G4 and H7, also during the final second (the first flash word is written last).

## Size

`tools/sizes.py` (Arm GNU Toolchain 14.3, `-Os`, LTO).  Stack is the deepest call chain from GCC's
call-graph output plus an exception frame; it is an upper bound.

| Board | Links | Flash | RAM | Stack |
| --- | --- | ---: | ---: | ---: |
| f042-can | DroneCAN | 7,468 B | 1,656 B | 388 B |
| f042-node | DroneCAN, SRXL2, serial (no LogMessage) | 11,576 B | 2,136 B | 444 B |
| f042-usb | USB | 7,804 B | 840 B | 316 B |
| f042-usb | USB, DroneCAN | 12,488 B | 2,176 B | 388 B |
| f042-node | all four | 15,244 B | 2,672 B | 444 B |
| f303-node | DroneCAN, serial, SRXL2 | 12,292 B | 2,144 B | 412 B |
| f303-usb | USB, serial, SRXL2 | 10,900 B | 1,332 B | 412 B |
| f405-node | all four | 16,228 B | 2,672 B | 412 B |
| g474-node | all four (CAN FD) | 16,604 B | 2,688 B | 816 B |
| matek-mr24-30 | USB, serial | 9,868 B | 1,168 B | 332 B |
| h743-node | all four | 16,504 B | 2,720 B | 432 B |
| h735-node | USB, DroneCAN (CAN FD), serial | 15,488 B | 2,576 B | 816 B |
| h735g-dk | USB | 8,688 B | 896 B | 308 B |
| tbs-lucid-h7 | USB, DroneCAN, serial | 15,160 B | 2,560 B | 432 B |
| tbs-lucid-h7 | USB | 8,932 B | 896 B | 316 B |
| f042, serial only | | 5,800 B | 640 B | 348 B |
| f042, SRXL2 only | | 6,176 B | 536 B | 436 B |

USB costs 3.7 KB on the F0 peripheral (driver, descriptors and a second protocol instance), and
3.5 KB (F4) to 3.6 KB (H7) on the OTG core.

On the F042, DroneCAN's LogMessage costs 520 B, RestartNode 184 B and the legacy CRC-64 format
256 B (`MUBL_DC_LOG`, `MUBL_DC_RESTART`, `MUBL_IMAGE_FORMATS`).  Cortex-M0 has no divide instruction;
`src/hal/stm32/divide.c` replaces libgcc's 276-byte division helpers with a 54-byte loop.

## Testing

`make test` builds the simulation (`src/hal/host`: NOR-flash semantics, ECC words, power-loss
injection, retained RAM, CAN with a bus bitrate, UART ports, models of the two USB peripherals and a
USB host that enumerates the device) as a shared library and runs:

| File | Covers | Needs |
| --- | --- | --- |
| `test_core.py` | boot policy, every image check, handoff and its CRC, boot-loop counting, ECC faults, the update engine on F0/F4/G4/H7 flash, power loss at every operation (also of raw images with `MUBL_RAW_IMAGES`), `app/mubl_app.h`, the Cortex-M0 divide against native division, the CRC-unit driver against a model of the unit, and boot, update and handoff with that model in the simulation (its clock stops in `hal_deinit`) | |
| `test_dronecan.py` | allocation, GetNodeInfo, autobaud, `FirmwareUpdater` end to end, AP_Periph and PX4 handoffs, RestartNode, rejected images, a file server lost mid-update, busy with another link | sfwa-dronecan checkout at `../dronecan` (`SFWA_DRONECAN=`) |
| `test_serial.py` | ArduPilot's `uploader.py` unmodified through a pseudo-terminal; protocol edge cases | `../ardupilot/Tools/scripts/uploader.py` (`UPLOADER_PY=`); a pty (the macOS sandbox denies them) |
| `test_srxl2.py` | handshake against Spektrum's reference slave, the update packet with `tools/mubl_srxl2.py` (its protocol class, and its command line through a pseudo-terminal), retries, 400 kbaud (also with answers lost while the device holds the fast rate) | a pty for the command-line test |
| `test_links.py` | shared ports, baud-rate slicing, priority, exclusive links, handoff-named links, traffic not holding the boot | |
| `test_usb.py` | µCAN's USB driver tests (`tests/usb_fs_test.c`) against both drivers; enumeration, staying while a host is there and booting when it goes, uploads over USB on F0/F4/H7 flash (also with blocking flash), the UART and USB protocol instances side by side, handoff to USB, re-enumeration; every scenario on the F0/F3 peripheral, the OTG core and the F405's OTG core; `uploader.py` through USB | a pty for `uploader.py` |
| `test_fuzz.py` | `tests/fuzz.c`: random and well-formed-but-random traffic on all four links (USB with the cable coming and going and the host suspending), random flash and handoff RAM, under AddressSanitizer and UBSan (with the `mem*` functions declared `nonnull` as glibc does, `tests/nonnull.h`), on both USB peripheral models (13,500 iterations clean before USB; reaches 95 % of the serial, 93 % of the SRXL2 and 83 % of the DroneCAN lines) | `MUBL_FUZZ_ITERATIONS=` |

`tests/shim/serial` is a minimal termios stand-in for pyserial so `uploader.py` runs without it.
`MUBL_EXHAUSTIVE=1` makes the power-loss tests visit every flash operation (about a minute).
`MUBL_SIM_BLOCKING=1` runs every test with time passing inside blocking calls (delays, UART and CAN
transmission, flash erase and program), as on hardware; some tests always do, and so does the fuzzer
half the time.

### Hardware tests

`tests/hw/h735g-dk-app` is a test application for the STM32H735G-DK (`make` there gives `.apj`
files for versions 1 and 2 with `APP_VERSION=`).  It reports on the ST-LINK's virtual COM port what
the bootloader handed over, ticks every second, and on a key resets (`r`) or asks the bootloader to
stay (`b`).  Run on a kit on 2026-11-25, with the bootloader flashed over SWD:

- the USB FS port enumerates (0483:5740, serial number = unique ID) and `uploader.py --identify`
  reports protocol 5, board 7352 and 896 KiB;
- `uploader.py` erases, programs and verifies the application, and it starts with the handoff
  saying USB; a second version replaces the first;
- with the host attached the bootloader stays after a reset (stay reason USB) until BOOT; the
  application's stay request is read (stay reason requested); LED1 is on in the bootloader;
- with the USB cable out, a reset starts the application after the boot delay (handoff: no link).

`tests/hw/usb-app` is a test application with a USB console instead, for boards without a debug
probe's serial port: it brings up USB with µBL's own driver (product "mubl test app"), and answers
`i` (version, VTOR, handoff, uptime), `r` and `b` there (`make BOARD=tbs-lucid-h7` or `h735g-dk`).
Run on a TBS Lucid H7 on 2026-11-25, with the bootloader and that application written through ST's
ROM DFU loader (BOOT button held at plug-in) and started from it:

- the bootloader enumerates as "TBS Lucid H7-BL" and stays; `uploader.py --identify` reports
  protocol 5, board 5250, 1664 KiB and an STM32H74x rev V;
- after BOOT the application enumerates as "mubl test app" and reports VTOR 0x08020000 and the
  handoff from USB;
- stock ArduCopter 4.7.0-beta3 (no descriptor: `MUBL_RAW_IMAGES`), 1.5 MB, uploads through
  `uploader.py` in 24 s and starts: it enumerates as ArduPilot's TBS_LUCID_H7 and sends heartbeats
  and AUTOPILOT_VERSION (4.7.0, board 5250), with the parameters in sectors 14-15 untouched.

## Limitations

- On hardware so far only USB on the STM32H735G-DK and a TBS Lucid H7; first bring-up elsewhere
  should watch clocks, CAN bit timing and the H7 supply.
- One CAN interface; no redundant interfaces.  DroneCAN only (no Cyphal).
- USB: no VBUS sensing (a suspended host counts as gone); full speed only; the device always
  re-enumerates for the application (no live handoff of the USB connection).  The USB models are
  written from the reference manuals, not checked against silicon.
- SRXL2 updates use µBL's own packet; no Spektrum tool speaks it.
- No signed images and no rollback: an interrupted update leaves the bootloader waiting for another.
- An erase blocks the main loop (up to a few seconds per H7 sector); links are not serviced meanwhile
  (USB control requests from the host wait for it).
