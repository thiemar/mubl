/*
 * STM32 registers used by µBL, by family.  Only what the bootloader touches is defined; the values
 * are checked against ST's CMSIS device headers by tests/regcheck (make regcheck).
 */
#ifndef MUBL_STM32_H
#define MUBL_STM32_H

#include "mubl.h"

#ifndef REG32
#define REG32(addr) (*(volatile uint32_t*)(uintptr_t)(addr))
#endif

/* ---- Cortex-M core ----------------------------------------------------------------------- */

#define SCB_ICSR REG32(0xE000ED04U)
#define SCB_VTOR REG32(0xE000ED08U)
#define SCB_AIRCR REG32(0xE000ED0CU)
#define SCB_SHCSR REG32(0xE000ED24U)
#define SCB_CFSR REG32(0xE000ED28U)
#define SCB_HFSR REG32(0xE000ED2CU)
#define SYST_CSR REG32(0xE000E010U)
#define SYST_RVR REG32(0xE000E014U)
#define SYST_CVR REG32(0xE000E018U)
#define NVIC_ICER(n) REG32(0xE000E180U + 4U * (n))
#define NVIC_ICPR(n) REG32(0xE000E280U + 4U * (n))

#define ICSR_PENDSTCLR (1U << 25)
#define ICSR_PENDSVCLR (1U << 27)
#define AIRCR_RESET (0x05FA0000U | (1U << 2))

/* ---- GPIO (F0, F3, F4, H7 share the layout) ---------------------------------------------- */

#define GPIO_MODER 0x00U
#define GPIO_OTYPER 0x04U
#define GPIO_OSPEEDR 0x08U
#define GPIO_PUPDR 0x0CU
#define GPIO_IDR 0x10U
#define GPIO_BSRR 0x18U
#define GPIO_AFRL 0x20U

/* ---- USART ------------------------------------------------------------------------------- */

/* v2 (F0, F3, H7): CR1 CR2 CR3 BRR ... ISR ICR RDR TDR */
#define USART_CR1 0x00U
#define USART_CR3 0x08U
#define USART_BRR 0x0CU
#define USART_ISR 0x1CU
#define USART_ICR 0x20U
#define USART_RDR 0x24U
#define USART_TDR 0x28U
#define USART_CR1_UE (1U << 0)
/* v1 (F4): SR DR BRR CR1 CR2 CR3 */
#define USART1_SR 0x00U
#define USART1_DR 0x04U
#define USART1_BRR 0x08U
#define USART1_CR1 0x0CU
#define USART1_CR3 0x14U
#define USART1_CR1_UE (1U << 13)
/* common bits */
#define USART_CR1_RE (1U << 2)
#define USART_CR1_TE (1U << 3)
#define USART_CR3_HDSEL (1U << 3)
#define USART_ISR_FE (1U << 1)
#define USART_ISR_NE (1U << 2)
#define USART_ISR_ORE (1U << 3)
#define USART_ISR_RXNE (1U << 5)
#define USART_ISR_TC (1U << 6)
#define USART_ISR_TXE (1U << 7)

/* ---- bxCAN (F0, F3, F4) ------------------------------------------------------------------ */

#define CAN_MCR 0x000U
#define CAN_MSR 0x004U
#define CAN_TSR 0x008U
#define CAN_RF0R 0x00CU
#define CAN_ESR 0x018U
#define CAN_BTR 0x01CU
#define CAN_TIR(n) (0x180U + 0x10U * (n))
#define CAN_TDTR(n) (0x184U + 0x10U * (n))
#define CAN_TDLR(n) (0x188U + 0x10U * (n))
#define CAN_TDHR(n) (0x18CU + 0x10U * (n))
#define CAN_RI0R 0x1B0U
#define CAN_RDT0R 0x1B4U
#define CAN_RDL0R 0x1B8U
#define CAN_RDH0R 0x1BCU
#define CAN_FMR 0x200U
#define CAN_FM1R 0x204U
#define CAN_FS1R 0x20CU
#define CAN_FFA1R 0x214U
#define CAN_FA1R 0x21CU
#define CAN_FR1(n) (0x240U + 8U * (n))
#define CAN_FR2(n) (0x244U + 8U * (n))

#define CAN_MCR_INRQ (1U << 0)
#define CAN_MCR_SLEEP (1U << 1)
#define CAN_MCR_TXFP (1U << 2)
#define CAN_MCR_ABOM (1U << 6)
#define CAN_MCR_RESET (1U << 15)
#define CAN_MSR_INAK (1U << 0)
#define CAN_TSR_ABRQ(n) (1U << (7U + 8U * (n)))
#define CAN_TSR_TME_ALL (7U << 26)
#define CAN_TSR_CODE_POS 24U
#define CAN_RF0R_FMP0 3U
#define CAN_RF0R_FOVR0 (1U << 4)
#define CAN_RF0R_RFOM0 (1U << 5)
#define CAN_ESR_LEC_POS 4U
#define CAN_BTR_SILM (1U << 31)
#define CAN_FMR_FINIT (1U << 0)
#define CAN_TIR_TXRQ (1U << 0)
#define CAN_TIR_IDE (1U << 2)
#define CAN_RIR_RTR (1U << 1)
#define CAN_RIR_IDE (1U << 2)

/* ---- FDCAN (H7, G4) ----------------------------------------------------------------------- */

#define FDCAN_DBTP 0x00CU
#define FDCAN_CCCR 0x018U
#define FDCAN_NBTP 0x01CU
#define FDCAN_PSR 0x044U
#define FDCAN_TDCR 0x048U
#define FDCAN_DBTP_TDC (1U << 23)
#if MUBL_FAMILY == MUBL_FAMILY_G4
/* STM32G4 FDCAN (RM0440 §44): fixed message RAM layout, compact register map. */
#define FDCAN_RXGFC 0x080U
#define FDCAN_XIDAM 0x084U
#define FDCAN_RXF0S 0x090U
#define FDCAN_RXF0A 0x094U
#define FDCAN_TXBC 0x0C0U
#define FDCAN_TXFQS 0x0C4U
#define FDCAN_TXBAR 0x0CCU
#define FDCAN_TXBCR 0x0D0U
#else
#define FDCAN_GFC 0x080U
#define FDCAN_SIDFC 0x084U
#define FDCAN_XIDFC 0x088U
#define FDCAN_XIDAM 0x090U
#define FDCAN_RXF0C 0x0A0U
#define FDCAN_RXF0S 0x0A4U
#define FDCAN_RXF0A 0x0A8U
#define FDCAN_RXESC 0x0BCU
#define FDCAN_TXBC 0x0C0U
#define FDCAN_TXFQS 0x0C4U
#define FDCAN_TXESC 0x0C8U
#define FDCAN_TXBAR 0x0D0U
#define FDCAN_TXBCR 0x0D4U
#endif

#define FDCAN_CCCR_INIT (1U << 0)
#define FDCAN_CCCR_CCE (1U << 1)
#define FDCAN_CCCR_MON (1U << 5)
#define FDCAN_CCCR_FDOE (1U << 8)
#define FDCAN_CCCR_BRSE (1U << 9)

/* ---- CRC unit (F0, F3, H7: input/output bit reversal and INIT; F4's has neither) ------------ */

#define CRC_DR 0x00U
#define CRC_CR 0x08U
#define CRC_INIT 0x10U
#define CRC_CR_RESET (1U << 0)
#define CRC_CR_REV_IN_BYTE (1U << 5)
#define CRC_CR_REV_OUT (1U << 7)

/* ---- Flash keys (all families) ------------------------------------------------------------ */

#define FLASH_KEY1 0x45670123U
#define FLASH_KEY2 0xCDEF89ABU

/* ---- IWDG --------------------------------------------------------------------------------- */

#define IWDG_KR 0x00U
#define IWDG_PR 0x04U
#define IWDG_RLR 0x08U
#define IWDG_SR 0x0CU

/* ---- Families ----------------------------------------------------------------------------- */

#if MUBL_FAMILY == MUBL_FAMILY_F0
#include "family_f0.h"
#elif MUBL_FAMILY == MUBL_FAMILY_F3
#include "family_f3.h"
#elif MUBL_FAMILY == MUBL_FAMILY_F4
#include "family_f4.h"
#elif MUBL_FAMILY == MUBL_FAMILY_G4
#include "family_g4.h"
#elif MUBL_FAMILY == MUBL_FAMILY_H7
#include "family_h7.h"
#else
#error "unsupported MUBL_FAMILY"
#endif
#ifndef STM32_CORTEX_M0
#define STM32_CORTEX_M0 0
#endif
#ifndef STM32_USART_V1
#define STM32_USART_V1 0
#endif

/* Instance tables from MUBL_UARTn_INSTANCE / MUBL_CAN_INSTANCE. */
#define MUBL_CAT3_(a, b, c) a##b##c
#define MUBL_CAT3(a, b, c) MUBL_CAT3_(a, b, c)

/* Clocks (clock.c) */
void stm32_clock_init(void);
void stm32_clock_deinit(void);
bool stm32_usb_clock(bool on);                      /* the 48 MHz USB clock and its trimming: false if none */
/* Pins (gpio.c) */
void stm32_pin_af(uint16_t pin, bool open_drain, bool pull_up);
void stm32_pin_output(uint16_t pin, bool level);
void stm32_pin_input(uint16_t pin, uint8_t pull);   /* 0 none, 1 up, 2 down */
bool stm32_pin_read(uint16_t pin);
void stm32_pin_write(uint16_t pin, bool level);
void stm32_pin_reset(uint16_t pin);                 /* back to analog/input as after reset */

#endif /* MUBL_STM32_H */
