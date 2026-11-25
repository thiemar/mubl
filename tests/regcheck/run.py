#!/usr/bin/env python3
"""Check µBL's STM32 register definitions against ST's CMSIS device headers.

    tests/regcheck/run.py [--cc arm-none-eabi-gcc] [--cmsis DIR]

For each part, our macros are expanded to plain numbers by the preprocessor (with REG32 turned
into "the address"), then compiled next to the vendor header as _Static_asserts.  The vendor
headers come from ChibiOS in the ArduPilot checkout next to this repository (CMSIS=dir to point
elsewhere: it needs ST/<family>/ and ARM/CMSIS/Core/Include/).
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CMSIS = os.path.join(ROOT, '..', 'ardupilot', 'modules', 'ChibiOS', 'os', 'common', 'ext')


def reg(ours, typ, base, field):
    return (ours, '(%s + offsetof(%s, %s))' % (base, typ, field))


def same(*names):
    return [(n, n) for n in names]


CORE = [
    ('&SCB_ICSR', '(SCB_BASE + offsetof(SCB_Type, ICSR))'),
    ('&SCB_AIRCR', '(SCB_BASE + offsetof(SCB_Type, AIRCR))'),
    ('&SYST_CSR', '(SysTick_BASE + offsetof(SysTick_Type, CTRL))'),
    ('&SYST_RVR', '(SysTick_BASE + offsetof(SysTick_Type, LOAD))'),
    ('&SYST_CVR', '(SysTick_BASE + offsetof(SysTick_Type, VAL))'),
    ('&NVIC_ICER(0)', '(NVIC_BASE + offsetof(NVIC_Type, ICER))'),
    ('&NVIC_ICPR(0)', '(NVIC_BASE + offsetof(NVIC_Type, ICPR))'),
    ('ICSR_PENDSTCLR', 'SCB_ICSR_PENDSTCLR_Msk'), ('ICSR_PENDSVCLR', 'SCB_ICSR_PENDSVCLR_Msk'),
    ('AIRCR_RESET', '((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk)'),
]
CORE_M3 = [('&SCB_VTOR', '(SCB_BASE + offsetof(SCB_Type, VTOR))'),
           ('&SCB_CFSR', '(SCB_BASE + offsetof(SCB_Type, CFSR))'),
           ('&SCB_HFSR', '(SCB_BASE + offsetof(SCB_Type, HFSR))')]

GPIO = [reg('GPIO_' + f, 'GPIO_TypeDef', '0', f) for f in ('MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'IDR', 'BSRR')] + \
    [('GPIO_AFRL', 'offsetof(GPIO_TypeDef, AFR)'), ('GPIO_BASE(0)', 'GPIOA_BASE'), ('GPIO_BASE(1)', 'GPIOB_BASE'),
     ('GPIO_BASE(2)', 'GPIOC_BASE')]

IWDG = [reg('IWDG_' + f, 'IWDG_TypeDef', '0', f) for f in ('KR', 'PR', 'RLR', 'SR')]

def usart_v2(rxne='USART_ISR_RXNE', txe='USART_ISR_TXE'):
    return [reg('USART_' + f, 'USART_TypeDef', '0', f) for f in ('CR1', 'CR3', 'BRR', 'ISR', 'ICR', 'RDR', 'TDR')] + \
        same('USART_CR1_UE', 'USART_CR1_RE', 'USART_CR1_TE', 'USART_CR3_HDSEL', 'USART_ISR_FE', 'USART_ISR_NE',
             'USART_ISR_ORE', 'USART_ISR_TC') + [('USART_ISR_RXNE', rxne), ('USART_ISR_TXE', txe)]


USART_V2 = usart_v2()

USART_V1 = [reg('USART1_' + f, 'USART_TypeDef', '0', f) for f in ('SR', 'DR', 'BRR', 'CR1', 'CR3')] + \
    [('USART1_CR1_UE', 'USART_CR1_UE'), ('USART_CR1_RE', 'USART_CR1_RE'), ('USART_CR1_TE', 'USART_CR1_TE'),
     ('USART_CR3_HDSEL', 'USART_CR3_HDSEL'), ('USART_ISR_FE', 'USART_SR_FE'), ('USART_ISR_NE', 'USART_SR_NE'),
     ('USART_ISR_ORE', 'USART_SR_ORE'), ('USART_ISR_RXNE', 'USART_SR_RXNE'), ('USART_ISR_TC', 'USART_SR_TC'),
     ('USART_ISR_TXE', 'USART_SR_TXE')]

BXCAN = [reg('CAN_' + f, 'CAN_TypeDef', '0', f) for f in ('MCR', 'MSR', 'TSR', 'RF0R', 'ESR', 'BTR', 'FMR', 'FM1R',
                                                           'FS1R', 'FFA1R', 'FA1R')] + \
    [('CAN_TIR(2)', 'offsetof(CAN_TypeDef, sTxMailBox[2].TIR)'), ('CAN_TDTR(1)', 'offsetof(CAN_TypeDef, sTxMailBox[1].TDTR)'),
     ('CAN_TDLR(1)', 'offsetof(CAN_TypeDef, sTxMailBox[1].TDLR)'), ('CAN_TDHR(2)', 'offsetof(CAN_TypeDef, sTxMailBox[2].TDHR)'),
     ('CAN_RI0R', 'offsetof(CAN_TypeDef, sFIFOMailBox[0].RIR)'), ('CAN_RDT0R', 'offsetof(CAN_TypeDef, sFIFOMailBox[0].RDTR)'),
     ('CAN_RDL0R', 'offsetof(CAN_TypeDef, sFIFOMailBox[0].RDLR)'), ('CAN_RDH0R', 'offsetof(CAN_TypeDef, sFIFOMailBox[0].RDHR)'),
     ('CAN_FR1(3)', 'offsetof(CAN_TypeDef, sFilterRegister[3].FR1)'), ('CAN_FR2(4)', 'offsetof(CAN_TypeDef, sFilterRegister[4].FR2)')] + \
    same('CAN_MCR_INRQ', 'CAN_MCR_SLEEP', 'CAN_MCR_TXFP', 'CAN_MCR_ABOM', 'CAN_MCR_RESET', 'CAN_MSR_INAK', 'CAN_RF0R_FMP0',
         'CAN_RF0R_FOVR0', 'CAN_RF0R_RFOM0', 'CAN_BTR_SILM', 'CAN_FMR_FINIT') + \
    [('CAN_TSR_CODE_POS', 'CAN_TSR_CODE_Pos'), ('CAN_ESR_LEC_POS', 'CAN_ESR_LEC_Pos'), ('CAN_TSR_TME_ALL', 'CAN_TSR_TME'),
     ('CAN_TSR_ABRQ(0)', 'CAN_TSR_ABRQ0'), ('CAN_TSR_ABRQ(1)', 'CAN_TSR_ABRQ1'), ('CAN_TSR_ABRQ(2)', 'CAN_TSR_ABRQ2'),
     ('CAN_TIR_TXRQ', 'CAN_TI0R_TXRQ'), ('CAN_TIR_IDE', 'CAN_TI0R_IDE'), ('CAN_RIR_RTR', 'CAN_RI0R_RTR'),
     ('CAN_RIR_IDE', 'CAN_RI0R_IDE')]

F0F3_RCC = [reg('&RCC_' + f, 'RCC_TypeDef', 'RCC_BASE', f) for f in ('CR', 'CFGR', 'APB2RSTR', 'APB1RSTR', 'AHBENR',
                                                                     'APB2ENR', 'APB1ENR', 'AHBRSTR', 'CFGR2')] + \
    same('RCC_CR_HSION', 'RCC_CR_HSIRDY', 'RCC_CR_HSEON', 'RCC_CR_HSERDY', 'RCC_CR_HSEBYP', 'RCC_CR_PLLON',
         'RCC_CR_PLLRDY', 'RCC_CFGR_PLLSRC_HSE_PREDIV') + \
    [('RCC_CFGR_SW_PLL', 'RCC_CFGR_SW_PLL'), ('RCC_CFGR_SWS_POS', 'RCC_CFGR_SWS_Pos'),
     ('RCC_CFGR_PLLMUL_POS', 'RCC_CFGR_PLLMUL_Pos'), ('STM32_RCC', 'RCC_BASE'), ('STM32_FLASH', 'FLASH_R_BASE'),
     ('STM32_IWDG', 'IWDG_BASE'), ('STM32_UID', 'UID_BASE'), ('CAN1_BASE', 'CAN_BASE'),
     ('CAN1_BIT', 'RCC_APB1ENR_CANEN'), ('1U << 17', 'RCC_AHBENR_GPIOAEN'), ('1U << 19', 'RCC_AHBENR_GPIOCEN'),
     ('USART1_BASE', 'USART1_BASE'), ('USART2_BASE', 'USART2_BASE'), ('USART1_BIT', 'RCC_APB2ENR_USART1EN'),
     ('USART2_BIT', 'RCC_APB1ENR_USART2EN')] + \
    [reg('&FLASH_' + f, 'FLASH_TypeDef', 'FLASH_R_BASE', f) for f in ('ACR', 'KEYR', 'SR', 'CR', 'AR')] + \
    same('FLASH_ACR_PRFTBE', 'FLASH_SR_BSY', 'FLASH_SR_PGERR', 'FLASH_SR_EOP', 'FLASH_CR_PG',
         'FLASH_CR_PER', 'FLASH_CR_STRT', 'FLASH_CR_LOCK') + [('FLASH_KEY1', 'FLASH_KEY1'), ('FLASH_KEY2', 'FLASH_KEY2'),
                                                              reg('&FLASH_OBR', 'FLASH_TypeDef', 'FLASH_R_BASE', 'OBR'),
                                                              ('FLASH_OBR_IWDG_SW', 'FLASH_OBR_IWDG_SW')]

USB_FS = [('USB_EPR(1)', 'offsetof(USB_TypeDef, EP1R)'), ('USB_EPR(3)', 'offsetof(USB_TypeDef, EP3R)')] + \
    [reg('USB_' + f, 'USB_TypeDef', '0', f) for f in ('CNTR', 'ISTR', 'DADDR', 'BTABLE')] + \
    same('USB_CNTR_FRES', 'USB_DADDR_EF', 'USB_ISTR_SOF', 'USB_ISTR_EP_ID') + \
    [('ISTR_CTR', 'USB_ISTR_CTR'), ('ISTR_RESET', 'USB_ISTR_RESET'), ('ISTR_SOF', 'USB_ISTR_SOF'),
     ('ISTR_EP_ID', 'USB_ISTR_EP_ID'),
     ('EPR_CTR_RX', 'USB_EP_CTR_RX'), ('EPR_DTOG_RX', 'USB_EP_DTOG_RX'), ('EPR_STAT_RX', 'USB_EPRX_STAT'),
     ('EPR_SETUP', 'USB_EP_SETUP'), ('EPR_TYPE', 'USB_EP_T_FIELD'), ('EPR_KIND', 'USB_EP_KIND'),
     ('EPR_CTR_TX', 'USB_EP_CTR_TX'), ('EPR_DTOG_TX', 'USB_EP_DTOG_TX'), ('EPR_STAT_TX', 'USB_EPTX_STAT'),
     ('EPR_EA', 'USB_EPADDR_FIELD'), ('EPR_TYPE_BULK', 'USB_EP_BULK'), ('EPR_TYPE_CONTROL', 'USB_EP_CONTROL'),
     ('EPR_TYPE_INTERRUPT', 'USB_EP_INTERRUPT'),
     ('RX_DIS', 'USB_EP_RX_DIS'), ('RX_STALL', 'USB_EP_RX_STALL'), ('RX_NAK', 'USB_EP_RX_NAK'),
     ('RX_VALID', 'USB_EP_RX_VALID'), ('TX_DIS', 'USB_EP_TX_DIS'), ('TX_STALL', 'USB_EP_TX_STALL'),
     ('TX_NAK', 'USB_EP_TX_NAK'), ('TX_VALID', 'USB_EP_TX_VALID'),
     ('STM32_USB', 'USB_BASE'), ('STM32_USB_PMA', 'USB_PMAADDR'), ('USB_CLOCK_BIT', 'RCC_APB1ENR_USBEN'),
     ('USB_CLOCK_BIT', 'RCC_APB1RSTR_USBRST'), ('&USB_CLOCK_ENR', '(RCC_BASE + offsetof(RCC_TypeDef, APB1ENR))'),
     ('&USB_RSTR', '(RCC_BASE + offsetof(RCC_TypeDef, APB1RSTR))')]


def otg(core_vbden):
    g = 'USB_OTG_GlobalTypeDef'
    checks = [reg('OTG_' + f, g, '0', f) for f in ('GOTGCTL', 'GAHBCFG', 'GUSBCFG', 'GRSTCTL', 'GINTSTS', 'GINTMSK',
                                                   'GRXSTSP', 'GRXFSIZ', 'GCCFG')] + \
        [('OTG_DIEPTXF0', 'offsetof(%s, DIEPTXF0_HNPTXFSIZ)' % g), ('OTG_DIEPTXF(1)', 'offsetof(%s, DIEPTXF[0])' % g),
         ('OTG_DIEPTXF(3)', 'offsetof(%s, DIEPTXF[2])' % g)] + \
        [('OTG_' + f, 'USB_OTG_DEVICE_BASE + offsetof(USB_OTG_DeviceTypeDef, %s)' % f)
         for f in ('DCFG', 'DCTL', 'DSTS', 'DIEPMSK', 'DOEPMSK', 'DAINTMSK')] + \
        [('OTG_DIEP%s(2)' % f, 'USB_OTG_IN_ENDPOINT_BASE + 2 * USB_OTG_EP_REG_SIZE + offsetof(USB_OTG_INEndpointTypeDef, DIEP%s)' % f)
         for f in ('CTL', 'INT', 'TSIZ')] + \
        [('OTG_DTXFSTS(3)', 'USB_OTG_IN_ENDPOINT_BASE + 3 * USB_OTG_EP_REG_SIZE + offsetof(USB_OTG_INEndpointTypeDef, DTXFSTS)')] + \
        [('OTG_DOEP%s(1)' % f, 'USB_OTG_OUT_ENDPOINT_BASE + USB_OTG_EP_REG_SIZE + offsetof(USB_OTG_OUTEndpointTypeDef, DOEP%s)' % f)
         for f in ('CTL', 'INT', 'TSIZ')] + \
        [('OTG_PCGCCTL', 'USB_OTG_PCGCCTL_BASE'), ('OTG_FIFO(2)', 'USB_OTG_FIFO_BASE + 2 * USB_OTG_FIFO_SIZE')] + \
        [('GUSBCFG_PHYSEL', 'USB_OTG_GUSBCFG_PHYSEL'), ('GUSBCFG_TRDT', 'USB_OTG_GUSBCFG_TRDT'),
         ('GUSBCFG_TRDT_POS', 'USB_OTG_GUSBCFG_TRDT_Pos'), ('GUSBCFG_FHMOD', 'USB_OTG_GUSBCFG_FHMOD'),
         ('GUSBCFG_FDMOD', 'USB_OTG_GUSBCFG_FDMOD'), ('GRSTCTL_CSRST', 'USB_OTG_GRSTCTL_CSRST'),
         ('GRSTCTL_RXFFLSH', 'USB_OTG_GRSTCTL_RXFFLSH'), ('GRSTCTL_TXFFLSH', 'USB_OTG_GRSTCTL_TXFFLSH'),
         ('GRSTCTL_TXFNUM_POS', 'USB_OTG_GRSTCTL_TXFNUM_Pos'), ('GRSTCTL_AHBIDL', 'USB_OTG_GRSTCTL_AHBIDL'),
         ('GINTSTS_CMOD', 'USB_OTG_GINTSTS_CMOD'), ('GINTSTS_SOF', 'USB_OTG_GINTSTS_SOF'),
         ('GINTSTS_RXFLVL', 'USB_OTG_GINTSTS_RXFLVL'), ('GINTSTS_USBRST', 'USB_OTG_GINTSTS_USBRST'),
         ('GINTSTS_ENUMDNE', 'USB_OTG_GINTSTS_ENUMDNE'), ('GINTSTS_GONAKEFF', 'USB_OTG_GINTSTS_BOUTNAKEFF'),
         ('GRXSTS_EPNUM', 'USB_OTG_GRXSTSP_EPNUM'), ('GRXSTS_BCNT', 'USB_OTG_GRXSTSP_BCNT'),
         ('GRXSTS_PKTSTS', 'USB_OTG_GRXSTSP_PKTSTS'), ('GRXSTS_PKTSTS_POS', 'USB_OTG_GRXSTSP_PKTSTS_Pos'),
         ('GCCFG_PWRDWN', 'USB_OTG_GCCFG_PWRDWN'),
         ('DCFG_DSPD_FS', 'USB_OTG_DCFG_DSPD'), ('DCFG_DAD', 'USB_OTG_DCFG_DAD'), ('DCFG_DAD_POS', 'USB_OTG_DCFG_DAD_Pos'),
         ('DCTL_SDIS', 'USB_OTG_DCTL_SDIS'), ('DCTL_CGINAK', 'USB_OTG_DCTL_CGINAK'), ('DCTL_SGONAK', 'USB_OTG_DCTL_SGONAK'),
         ('DCTL_CGONAK', 'USB_OTG_DCTL_CGONAK'),
         ('EPCTL_MPSIZ', 'USB_OTG_DIEPCTL_MPSIZ'), ('EPCTL_USBAEP', 'USB_OTG_DIEPCTL_USBAEP'),
         ('EPCTL_NAKSTS', 'USB_OTG_DIEPCTL_NAKSTS'), ('EPCTL_EPTYP_POS', 'USB_OTG_DIEPCTL_EPTYP_Pos'),
         ('EPCTL_STALL', 'USB_OTG_DIEPCTL_STALL'), ('EPCTL_TXFNUM_POS', 'USB_OTG_DIEPCTL_TXFNUM_Pos'),
         ('EPCTL_CNAK', 'USB_OTG_DIEPCTL_CNAK'), ('EPCTL_SNAK', 'USB_OTG_DIEPCTL_SNAK'),
         ('EPCTL_SD0PID', 'USB_OTG_DIEPCTL_SD0PID_SEVNFRM'), ('EPCTL_EPDIS', 'USB_OTG_DIEPCTL_EPDIS'),
         ('EPCTL_EPENA', 'USB_OTG_DIEPCTL_EPENA'), ('EPCTL_STALL', 'USB_OTG_DOEPCTL_STALL'),
         ('EPCTL_SD0PID', 'USB_OTG_DOEPCTL_SD0PID_SEVNFRM'), ('EPCTL_EPENA', 'USB_OTG_DOEPCTL_EPENA'),
         ('EPCTL_EPDIS', 'USB_OTG_DOEPCTL_EPDIS'), ('EPCTL_CNAK', 'USB_OTG_DOEPCTL_CNAK'),
         ('EPCTL_SNAK', 'USB_OTG_DOEPCTL_SNAK'), ('EPCTL_USBAEP', 'USB_OTG_DOEPCTL_USBAEP'),
         ('EPINT_XFRC', 'USB_OTG_DIEPINT_XFRC'), ('EPINT_EPDISD', 'USB_OTG_DIEPINT_EPDISD'),
         ('EPINT_XFRC', 'USB_OTG_DOEPINT_XFRC'), ('EPINT_EPDISD', 'USB_OTG_DOEPINT_EPDISD'),
         ('DOEPINT_STUP', 'USB_OTG_DOEPINT_STUP'), ('DIEPINT_INEPNE', 'USB_OTG_DIEPINT_INEPNE'),
         ('EPTSIZ_PKTCNT_POS', 'USB_OTG_DIEPTSIZ_PKTCNT_Pos'), ('EPTSIZ_PKTCNT_POS', 'USB_OTG_DOEPTSIZ_PKTCNT_Pos'),
         ('DOEPTSIZ0_STUPCNT', 'USB_OTG_DOEPTSIZ_STUPCNT'), ('DTXFSTS_INEPTFSAV', 'USB_OTG_DTXFSTS_INEPTFSAV')]
    if core_vbden:
        checks += [('GCCFG_VBDEN', 'USB_OTG_GCCFG_VBDEN'), ('GOTGCTL_BVALOEN', 'USB_OTG_GOTGCTL_BVALOEN'),
                   ('GOTGCTL_BVALOVAL', 'USB_OTG_GOTGCTL_BVALOVAL')]
    else:
        checks += [('GCCFG_NOVBUSSENS', 'USB_OTG_GCCFG_NOVBUSSENS')]
    return checks


def crc_unit(enr, bit):
    return [('STM32_CRC', 'CRC_BASE'), reg('CRC_DR', 'CRC_TypeDef', '0', 'DR'), reg('CRC_CR', 'CRC_TypeDef', '0', 'CR'),
            reg('CRC_INIT', 'CRC_TypeDef', '0', 'INIT'), ('CRC_CR_RESET', 'CRC_CR_RESET'),
            ('CRC_CR_REV_IN_BYTE', 'CRC_CR_REV_IN_0'), ('CRC_CR_REV_OUT', 'CRC_CR_REV_OUT'),
            ('CRC_CLOCK_BIT', bit), ('&CRC_CLOCK_ENR', '(RCC_BASE + offsetof(RCC_TypeDef, %s))' % enr)]


PARTS = {
    'stm32f042x6': ('F0', 'STM32F0xx', 'core_cm0',
                    CORE + GPIO + IWDG + USART_V2 + BXCAN + F0F3_RCC + [
                        ('FLASH_SR_WRPRTERR', 'FLASH_SR_WRPRTERR'),
                        reg('&RCC_CR2', 'RCC_TypeDef', 'RCC_BASE', 'CR2'),
                        ('RCC_CR2_HSI48ON', 'RCC_CR2_HSI48ON'), ('RCC_CR2_HSI48RDY', 'RCC_CR2_HSI48RDY'),
                        ('RCC_CFGR_SW_HSI48', 'RCC_CFGR_SW_HSI48'), ('RCC_APB2ENR_SYSCFGEN', 'RCC_APB2ENR_SYSCFGCOMPEN'),
                        ('STM32_SYSCFG', 'SYSCFG_BASE'), ('SYSCFG_CFGR1_MEM_MODE', 'SYSCFG_CFGR1_MEM_MODE'),
                        ('SYSCFG_CFGR1_PA11_PA12_RMP', 'SYSCFG_CFGR1_PA11_PA12_RMP'),
                        ('STM32_IDCODE', '(DBGMCU_BASE + offsetof(DBGMCU_TypeDef, IDCODE))')] + USB_FS + [
                        reg('USB_BCDR', 'USB_TypeDef', '0', 'BCDR'), ('USB_BCDR_DPPU', 'USB_BCDR_DPPU'),
                        reg('&RCC_CFGR3', 'RCC_TypeDef', 'RCC_BASE', 'CFGR3'), ('RCC_CFGR3_USBSW', 'RCC_CFGR3_USBSW'),
                        ('STM32_CRS', 'CRS_BASE'), reg('&CRS_CR', 'CRS_TypeDef', 'CRS_BASE', 'CR'),
                        ('CRS_CR_CEN', 'CRS_CR_CEN'), ('CRS_CR_AUTOTRIMEN', 'CRS_CR_AUTOTRIMEN'),
                        ('CRS_BIT', 'RCC_APB1ENR_CRSEN'), ('CRS_BIT', 'RCC_APB1RSTR_CRSRST')] +
                    crc_unit('AHBENR', 'RCC_AHBENR_CRCEN')),
    'stm32f303xc': ('F3', 'STM32F3xx', 'core_cm4',
                    CORE + CORE_M3 + GPIO + IWDG + USART_V2 + BXCAN + F0F3_RCC + [
                        ('FLASH_SR_WRPRTERR', 'FLASH_SR_WRPERR'),
                        ('RCC_CFGR_PPRE1_POS', 'RCC_CFGR_PPRE1_Pos'), ('RCC_CFGR_PPRE2_POS', 'RCC_CFGR_PPRE2_Pos'),
                        ('USART3_BASE', 'USART3_BASE'), ('USART3_BIT', 'RCC_APB1ENR_USART3EN'),
                        ('USART4_BASE', 'UART4_BASE'), ('USART4_BIT', 'RCC_APB1ENR_UART4EN'),
                        ('USART5_BASE', 'UART5_BASE'), ('USART5_BIT', 'RCC_APB1ENR_UART5EN'),
                        ('STM32_IDCODE', '(DBGMCU_BASE + offsetof(DBGMCU_TypeDef, IDCODE))')] + USB_FS +
                    crc_unit('AHBENR', 'RCC_AHBENR_CRCEN')),
}

F4 = CORE + CORE_M3 + GPIO + IWDG + USART_V1 + BXCAN + \
    [reg('&RCC_' + f, 'RCC_TypeDef', 'RCC_BASE', f) for f in ('CR', 'PLLCFGR', 'CFGR', 'AHB1RSTR', 'APB1RSTR',
                                                              'APB2RSTR', 'AHB1ENR', 'APB1ENR', 'APB2ENR')] + \
    same('RCC_CR_HSION', 'RCC_CR_HSIRDY', 'RCC_CR_HSEON', 'RCC_CR_HSERDY', 'RCC_CR_HSEBYP', 'RCC_CR_PLLON',
         'RCC_CR_PLLRDY', 'RCC_PLLCFGR_PLLSRC_HSE', 'FLASH_ACR_PRFTEN', 'FLASH_ACR_ICEN', 'FLASH_ACR_DCEN',
         'FLASH_ACR_ICRST', 'FLASH_ACR_DCRST', 'FLASH_SR_BSY', 'FLASH_CR_PG', 'FLASH_CR_SER', 'FLASH_CR_STRT',
         'FLASH_CR_LOCK') + \
    [('RCC_PLLCFGR_PLLN_POS', 'RCC_PLLCFGR_PLLN_Pos'), ('RCC_PLLCFGR_PLLP_POS', 'RCC_PLLCFGR_PLLP_Pos'),
     ('RCC_PLLCFGR_PLLQ_POS', 'RCC_PLLCFGR_PLLQ_Pos'), ('RCC_CFGR_SW_PLL', 'RCC_CFGR_SW_PLL'),
     ('RCC_CFGR_SWS_POS', 'RCC_CFGR_SWS_Pos'), ('RCC_CFGR_PPRE1_POS', 'RCC_CFGR_PPRE1_Pos'),
     ('RCC_CFGR_PPRE2_POS', 'RCC_CFGR_PPRE2_Pos'), ('FLASH_CR_SNB_POS', 'FLASH_CR_SNB_Pos'),
     ('FLASH_CR_PSIZE_POS', 'FLASH_CR_PSIZE_Pos'),
     ('FLASH_SR_ERRORS', '(FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)'),
     ('STM32_RCC', 'RCC_BASE'), ('STM32_FLASH', 'FLASH_R_BASE'), ('STM32_IWDG', 'IWDG_BASE'), ('STM32_UID', 'UID_BASE'),
     ('STM32_IDCODE', '(DBGMCU_BASE + offsetof(DBGMCU_TypeDef, IDCODE))'),
     ('CAN1_BASE', 'CAN1_BASE'), ('CAN2_BASE', 'CAN2_BASE'), ('CAN1_BIT', 'RCC_APB1ENR_CAN1EN'),
     ('CAN2_BIT', '(RCC_APB1ENR_CAN2EN | RCC_APB1ENR_CAN1EN)'), ('1U << 3', 'RCC_AHB1ENR_GPIODEN'),
     ('USART1_BASE', 'USART1_BASE'), ('USART2_BASE', 'USART2_BASE'), ('USART3_BASE', 'USART3_BASE'),
     ('USART4_BASE', 'UART4_BASE'), ('USART5_BASE', 'UART5_BASE'), ('USART6_BASE', 'USART6_BASE'),
     ('USART1_BIT', 'RCC_APB2ENR_USART1EN'), ('USART6_BIT', 'RCC_APB2ENR_USART6EN'),
     ('USART2_BIT', 'RCC_APB1ENR_USART2EN'), ('USART3_BIT', 'RCC_APB1ENR_USART3EN'),
     ('USART4_BIT', 'RCC_APB1ENR_UART4EN'), ('USART5_BIT', 'RCC_APB1ENR_UART5EN')] + \
    [reg('&FLASH_' + f, 'FLASH_TypeDef', 'FLASH_R_BASE', f) for f in ('ACR', 'KEYR', 'SR', 'CR', 'OPTCR')] + \
    [('FLASH_OPTCR_WDG_SW', 'FLASH_OPTCR_WDG_SW')]
F4 += [('STM32_OTG', 'USB_OTG_FS_PERIPH_BASE'), ('OTG_CLOCK_BIT', 'RCC_AHB2ENR_OTGFSEN'),
       ('OTG_CLOCK_BIT', 'RCC_AHB2RSTR_OTGFSRST'), ('&RCC_AHB2ENR', '(RCC_BASE + offsetof(RCC_TypeDef, AHB2ENR))'),
       ('&RCC_AHB2RSTR', '(RCC_BASE + offsetof(RCC_TypeDef, AHB2RSTR))')]
PARTS['stm32f405xx'] = ('F4', 'STM32F4xx', 'core_cm4', F4 + otg(False) + [('RCC_PLLCFGR_RESET', '0x24003010U')])
PARTS['stm32f446xx'] = ('F4', 'STM32F4xx', 'core_cm4', F4 + otg(True) + [('RCC_PLLCFGR_RESET & 0x70000000U', 'RCC_PLLCFGR_PLLR_1')])

FDCAN = [reg('FDCAN_' + f, 'FDCAN_GlobalTypeDef', '0', f) for f in (
    'CCCR', 'NBTP', 'DBTP', 'PSR', 'GFC', 'SIDFC', 'XIDFC', 'XIDAM', 'RXF0C', 'RXF0S', 'RXF0A', 'RXESC', 'TXBC',
    'TXFQS', 'TXESC', 'TXBAR', 'TXBCR')] + \
    same('FDCAN_CCCR_INIT', 'FDCAN_CCCR_CCE', 'FDCAN_CCCR_MON', 'FDCAN_CCCR_FDOE', 'FDCAN_CCCR_BRSE') + \
    [('FDCAN1_BASE', 'FDCAN1_BASE'), ('FDCAN2_BASE', 'FDCAN2_BASE'), ('FDCAN_RAM', 'SRAMCAN_BASE'),
     reg('FDCAN_TDCR', 'FDCAN_GlobalTypeDef', '0', 'TDCR'), ('FDCAN_DBTP_TDC', 'FDCAN_DBTP_TDC'),
     ('8U', 'FDCAN_TDCR_TDCO_Pos'), ('7U', 'FDCAN_PSR_BO_Pos')]
H7 = CORE + CORE_M3 + GPIO + IWDG + usart_v2('USART_ISR_RXNE_RXFNE', 'USART_ISR_TXE_TXFNF') + FDCAN + \
    [reg('&RCC_' + f, 'RCC_TypeDef', 'RCC_BASE', f) for f in ('CR', 'CFGR', 'D2CCIP1R', 'AHB4RSTR', 'APB1LRSTR',
                                                              'APB1HRSTR', 'APB2RSTR', 'AHB4ENR', 'APB1LENR',
                                                              'APB1HENR', 'APB2ENR', 'AHB2ENR')] + \
    [reg('&PWR_' + f, 'PWR_TypeDef', 'PWR_BASE', f) for f in ('CSR1', 'CR3')] + \
    [('&FLASH_ACR', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, ACR))'),
     ('&FLASH_KEYR(0)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, KEYR1))'),
     ('&FLASH_CR(0)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, CR1))'),
     ('&FLASH_SR(0)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, SR1))'),
     ('&FLASH_CCR(0)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, CCR1))'),
     ('&FLASH_OPTSR_CUR', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, OPTSR_CUR))'),
     ('FLASH_OPTSR_IWDG1_SW', 'FLASH_OPTSR_IWDG1_SW'), ('FLASH_CR_PSIZE_POS', 'FLASH_CR_PSIZE_Pos')] + \
    same('RCC_CR_HSEON', 'RCC_CR_HSERDY', 'RCC_CR_HSEBYP', 'PWR_CSR1_ACTVOSRDY', 'FLASH_CR_LOCK', 'FLASH_CR_PG',
         'FLASH_CR_SER', 'FLASH_CR_START', 'FLASH_SR_BSY', 'FLASH_SR_WBNE', 'FLASH_SR_QW', 'FLASH_SR_DBECCERR',
         'RCC_AHB2ENR_SRAM1EN', 'RCC_AHB2ENR_SRAM2EN') + \
    [('RCC_D2CCIP1R_FDCANSEL_POS', 'RCC_D2CCIP1R_FDCANSEL_Pos'), ('FLASH_ACR_LATENCY_POS', 'FLASH_ACR_LATENCY_Pos'),
     ('FLASH_ACR_WRHIGHFREQ_POS', 'FLASH_ACR_WRHIGHFREQ_Pos'), ('FLASH_CR_SNB_POS', 'FLASH_CR_SNB_Pos'),
     ('FLASH_SR_PROG_ERRORS', '(FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR)'),
     ('FLASH_CCR_ALL', '(FLASH_CCR_CLR_EOP | FLASH_CCR_CLR_WRPERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_STRBERR | '
                       'FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_OPERR | FLASH_CCR_CLR_RDPERR | FLASH_CCR_CLR_RDSERR | '
                       'FLASH_CCR_CLR_SNECCERR | FLASH_CCR_CLR_DBECCERR | FLASH_CCR_CLR_CRCEND)'),
     ('STM32_RCC', 'RCC_BASE'), ('STM32_PWR', 'PWR_BASE'), ('STM32_FLASH', 'FLASH_R_BASE'),
     ('STM32_IWDG', 'IWDG1_BASE'), ('STM32_UID', 'UID_BASE'), ('STM32_IDCODE', 'DBGMCU_BASE'),
     ('FDCAN_BIT', 'RCC_APB1HENR_FDCANEN'), ('1U << 3', 'RCC_AHB4ENR_GPIODEN'),
     ('USART1_BASE', 'USART1_BASE'), ('USART2_BASE', 'USART2_BASE'), ('USART3_BASE', 'USART3_BASE'),
     ('USART4_BASE', 'UART4_BASE'), ('USART5_BASE', 'UART5_BASE'), ('USART6_BASE', 'USART6_BASE'),
     ('USART7_BASE', 'UART7_BASE'), ('USART8_BASE', 'UART8_BASE'),
     ('USART1_BIT', 'RCC_APB2ENR_USART1EN'), ('USART6_BIT', 'RCC_APB2ENR_USART6EN'),
     ('USART2_BIT', 'RCC_APB1LENR_USART2EN'), ('USART3_BIT', 'RCC_APB1LENR_USART3EN'),
     ('USART4_BIT', 'RCC_APB1LENR_UART4EN'), ('USART5_BIT', 'RCC_APB1LENR_UART5EN'),
     ('USART7_BIT', 'RCC_APB1LENR_UART7EN'), ('USART8_BIT', 'RCC_APB1LENR_UART8EN')] + \
    crc_unit('AHB4ENR', 'RCC_AHB4ENR_CRCEN')
H7 += otg(True) + [
    reg('&RCC_' + f, 'RCC_TypeDef', 'RCC_BASE', f) for f in ('D2CCIP2R', 'AHB1RSTR', 'AHB1ENR')] + [
    ('RCC_D2CCIP2R_USBSEL', 'RCC_D2CCIP2R_USBSEL'), ('RCC_CR_HSI48ON', 'RCC_CR_HSI48ON'), ('RCC_CR_HSI48RDY', 'RCC_CR_HSI48RDY'),
    ('PWR_CR3_USB33DEN', 'PWR_CR3_USB33DEN'), ('PWR_CR3_USB33RDY', 'PWR_CR3_USB33RDY'),
    ('STM32_CRS', 'CRS_BASE'), reg('&CRS_CR', 'CRS_TypeDef', 'CRS_BASE', 'CR'), reg('&CRS_CFGR', 'CRS_TypeDef', 'CRS_BASE', 'CFGR'),
    ('CRS_CR_CEN', 'CRS_CR_CEN'), ('CRS_CR_AUTOTRIMEN', 'CRS_CR_AUTOTRIMEN'), ('CRS_CFGR_SYNCSRC_POS', 'CRS_CFGR_SYNCSRC_Pos'),
    ('CRS_BIT', 'RCC_APB1HENR_CRSEN'), ('CRS_BIT', 'RCC_APB1HRSTR_CRSRST'),
    ('STM32_OTG', 'USB1_OTG_HS_PERIPH_BASE'), ('OTG_CLOCK_BIT', 'RCC_AHB1ENR_USB1OTGHSEN'),
    ('OTG_CLOCK_BIT', 'RCC_AHB1RSTR_USB1OTGHSRST'), ('CRS_SYNC_USB << CRS_CFGR_SYNCSRC_POS', 'CRS_CFGR_SYNCSRC_1')]
PARTS['stm32h743xx'] = ('H7', 'STM32H7xx', 'core_cm7', H7 + [
    ('&FLASH_KEYR(1)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, KEYR2))'),
    ('&FLASH_CR(1)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, CR2))'),
    ('&FLASH_SR(1)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, SR2))'),
    ('&FLASH_CCR(1)', '(FLASH_R_BASE + offsetof(FLASH_TypeDef, CCR2))'), ('0x2', 'PWR_CR3_LDOEN'),
    ('RCC_AHB2ENR_SRAM3EN', 'RCC_AHB2ENR_SRAM3EN')])
PARTS['stm32h735xx'] = ('H7', 'STM32H7xx', 'core_cm7', H7 + [('0x4', 'PWR_CR3_SMPSEN')])
# The H74x's second core, USB2 OTG_FS (part:defines checks the same header with other board settings).
PARTS['stm32h743xx:MUBL_USB_OTG_INSTANCE=2'] = ('H7', 'STM32H7xx', 'core_cm7', [
    ('STM32_OTG', 'USB2_OTG_FS_PERIPH_BASE'), ('OTG_CLOCK_BIT', 'RCC_AHB1ENR_USB2OTGFSEN'),
    ('OTG_CLOCK_BIT', 'RCC_AHB1RSTR_USB2OTGFSRST'),
    ('CRS_SYNC_USB << CRS_CFGR_SYNCSRC_POS', '(CRS_CFGR_SYNCSRC_1 | CRS_CFGR_SYNCSRC_0)')])

BOARD = '''#define MUBL_FAMILY MUBL_FAMILY_%s
#define MUBL_BL_SIZE 0x20000
#define MUBL_APP_SIZE 0x20000
#define MUBL_RAM_BASE 0x20000000
#define MUBL_RAM_SIZE 0x8000
#define MUBL_DRONECAN 1
#define MUBL_H7_PWR_CR3 2
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cc', default='arm-none-eabi-gcc')
    ap.add_argument('--cmsis', default=os.environ.get('CMSIS', DEFAULT_CMSIS))
    args = ap.parse_args()
    if not os.path.isdir(args.cmsis):
        print('CMSIS headers not found at %s; skipping' % args.cmsis)
        return 0
    tmp = tempfile.mkdtemp()
    failed = 0
    for name, (family, folder, core, checks) in sorted(PARTS.items()):
        part, _, extra = name.partition(':')
        with open(os.path.join(tmp, 'board.h'), 'w') as f:
            f.write(BOARD % family + ''.join('#define %s %s\n' % tuple(d.split('=')) for d in extra.split(',') if d))
        # 1. our side, as numbers
        marker = '\n'.join('@@%d@@ (%s)' % (i, ours[1:] if ours.startswith('&') else ours)
                           for i, (ours, _) in enumerate(checks))
        src = os.path.join(tmp, 'ours.c')
        with open(src, 'w') as f:
            f.write('#define REG32(a) (a)\n#include "stm32.h"\n#include "hal/usb/usb_fs_regs.h"\n'
                    '#include "hal/usb/usb_otg_regs.h"\n' + marker + '\n')
        out = subprocess.check_output([args.cc, '-E', '-P', '-I' + tmp, '-I' + os.path.join(ROOT, 'src'),
                                       '-I' + os.path.join(ROOT, 'src', 'hal', 'stm32'), src]).decode()
        values = {}
        for line in out.splitlines():
            if line.startswith('@@'):
                i, expr = line[2:].split('@@', 1)
                values[int(i)] = expr.strip()
        # 2. compared with the vendor header
        body = ['#include <stddef.h>', '#include "%s.h"' % part]
        for i, (ours, theirs) in enumerate(checks):
            body.append('_Static_assert((unsigned long)(%s) == (unsigned long)(%s), "%s: %s != %s");'
                        % (values[i], theirs, part, ours.replace('"', ''), theirs.replace('"', '')))
        check = os.path.join(tmp, 'check.c')
        with open(check, 'w') as f:
            f.write('\n'.join(body) + '\n')
        r = subprocess.run([args.cc, '-fsyntax-only', '-std=c11', '-mthumb', '-I' + os.path.join(args.cmsis, 'ST', folder),
                            '-I' + os.path.join(args.cmsis, 'ARM', 'CMSIS', 'Core', 'Include'), check],
                           capture_output=True, text=True)
        errors = [l for l in r.stderr.splitlines() if 'static assertion failed' in l or 'error' in l]
        print('%-12s %3d checks: %s' % (name, len(checks), 'ok' if r.returncode == 0 else 'FAILED'))
        for e in errors[:20]:
            print('    ' + e.split('error: ', 1)[-1])
        failed += r.returncode != 0
    shutil.rmtree(tmp)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
