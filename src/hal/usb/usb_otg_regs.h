/*
 * The USB OTG core (Synopsys DWC2: OTG_FS on F4, OTG_HS/OTG_FS with the embedded full-speed PHY on
 * H7) in device mode: register offsets from the core's base and bits, for usb_otg.c.  Checked
 * against ST's headers by tests/regcheck.
 */
#ifndef MUBL_USB_OTG_REGS_H
#define MUBL_USB_OTG_REGS_H

#define OTG_GOTGCTL 0x000U
#define OTG_GAHBCFG 0x008U
#define OTG_GUSBCFG 0x00CU
#define OTG_GRSTCTL 0x010U
#define OTG_GINTSTS 0x014U
#define OTG_GINTMSK 0x018U
#define OTG_GRXSTSP 0x020U
#define OTG_GRXFSIZ 0x024U
#define OTG_DIEPTXF0 0x028U                        /* GNPTXFSIZ in host mode */
#define OTG_GCCFG 0x038U
#define OTG_DIEPTXF(n) (0x104U + 4U * ((n) - 1U))  /* n >= 1 */
#define OTG_DCFG 0x800U
#define OTG_DCTL 0x804U
#define OTG_DSTS 0x808U
#define OTG_DIEPMSK 0x810U
#define OTG_DOEPMSK 0x814U
#define OTG_DAINTMSK 0x81CU
#define OTG_DIEPCTL(n) (0x900U + 0x20U * (n))
#define OTG_DIEPINT(n) (0x908U + 0x20U * (n))
#define OTG_DIEPTSIZ(n) (0x910U + 0x20U * (n))
#define OTG_DTXFSTS(n) (0x918U + 0x20U * (n))
#define OTG_DOEPCTL(n) (0xB00U + 0x20U * (n))
#define OTG_DOEPINT(n) (0xB08U + 0x20U * (n))
#define OTG_DOEPTSIZ(n) (0xB10U + 0x20U * (n))
#define OTG_PCGCCTL 0xE00U
#define OTG_FIFO(n) (0x1000U + 0x1000U * (n))

#define GOTGCTL_BVALOEN (1U << 6)                  /* cores with GCCFG.VBDEN (F446, H7) */
#define GOTGCTL_BVALOVAL (1U << 7)
#define GUSBCFG_PHYSEL (1U << 6)
#define GUSBCFG_TRDT_POS 10U
#define GUSBCFG_TRDT (0xFU << 10)
#define GUSBCFG_FHMOD (1U << 29)
#define GUSBCFG_FDMOD (1U << 30)
#define GRSTCTL_CSRST (1U << 0)
#define GRSTCTL_RXFFLSH (1U << 4)
#define GRSTCTL_TXFFLSH (1U << 5)
#define GRSTCTL_TXFNUM_POS 6U
#define GRSTCTL_AHBIDL (1U << 31)
#define GINTSTS_CMOD (1U << 0)
#define GINTSTS_SOF (1U << 3)
#define GINTSTS_RXFLVL (1U << 4)
#define GINTSTS_USBRST (1U << 12)
#define GINTSTS_ENUMDNE (1U << 13)
#define GRXSTS_EPNUM 0xFU
#define GRXSTS_BCNT_POS 4U
#define GRXSTS_BCNT (0x7FFU << 4)
#define GRXSTS_PKTSTS_POS 17U
#define GRXSTS_PKTSTS (0xFU << 17)
#define PKTSTS_OUT_DATA 2U
#define PKTSTS_SETUP_DATA 6U
#define GCCFG_PWRDWN (1U << 16)
#define GCCFG_NOVBUSSENS (1U << 21)                /* F405/F407 core */
#define GCCFG_VBDEN (1U << 21)                     /* F446, H7 core */
#define DCFG_DSPD_FS 3U                            /* full speed, embedded PHY */
#define DCFG_DAD_POS 4U
#define DCFG_DAD (0x7FU << 4)
#define DCTL_SDIS (1U << 1)
#define DCTL_CGINAK (1U << 8)
#define DCTL_SGONAK (1U << 9)
#define DCTL_CGONAK (1U << 10)
#define GINTSTS_GONAKEFF (1U << 7)

#define EPCTL_MPSIZ 0x7FFU
#define EPCTL_USBAEP (1U << 15)
#define EPCTL_NAKSTS (1U << 17)
#define EPCTL_EPTYP_POS 18U
#define EPCTL_STALL (1U << 21)
#define EPCTL_TXFNUM_POS 22U
#define EPCTL_CNAK (1U << 26)
#define EPCTL_SNAK (1U << 27)
#define EPCTL_SD0PID (1U << 28)
#define EPCTL_EPDIS (1U << 30)
#define EPCTL_EPENA (1U << 31)
#define EPTYP_BULK 2U
#define EPTYP_INTERRUPT 3U

#define EPINT_XFRC (1U << 0)
#define EPINT_EPDISD (1U << 1)
#define DOEPINT_STUP (1U << 3)
#define DIEPINT_INEPNE (1U << 6)

#define EPTSIZ_PKTCNT_POS 19U
#define DOEPTSIZ0_STUPCNT (3U << 29)
#define DTXFSTS_INEPTFSAV 0xFFFFU

#endif
