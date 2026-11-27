/* STM32G4 (RM0440: G431/G441 Category 2, G473/G474/G483/G484 Category 3).
 *
 * Clocks stay simple: CPU and buses stay on HSI16 (16 MHz, all prescalers 1, 0 Flash wait states,
 * as after reset), USARTs run on PCLK (16 MHz -> 115200 at 0.08 % error, 400000 exact for SRXL2),
 * FDCAN runs directly from HSE, and USB FS runs from HSI48 trimmed by the CRS against USB SOF
 * (APB1 16 MHz meets the > 10 MHz minimum required by the USB peripheral).
 *
 * PWR_CR3_UCPD_DBDIS is set at start-up to disable the UCPD dead-battery pull-down resistors on
 * PA9 and PB6 (important when USART1_TX uses PA9).
 */
#ifndef MUBL_FAMILY_G4_H
#define MUBL_FAMILY_G4_H

#define STM32_RCC 0x40021000U
#define RCC_CR REG32(STM32_RCC + 0x00U)
#define RCC_CFGR REG32(STM32_RCC + 0x08U)
#define RCC_AHB1RSTR REG32(STM32_RCC + 0x28U)
#define RCC_AHB2RSTR REG32(STM32_RCC + 0x2CU)
#define RCC_APB1RSTR1 REG32(STM32_RCC + 0x38U)
#define RCC_APB1RSTR2 REG32(STM32_RCC + 0x3CU)
#define RCC_APB2RSTR REG32(STM32_RCC + 0x40U)
#define RCC_AHB1ENR REG32(STM32_RCC + 0x48U)
#define RCC_AHB2ENR REG32(STM32_RCC + 0x4CU)
#define RCC_APB1ENR1 REG32(STM32_RCC + 0x58U)
#define RCC_APB1ENR2 REG32(STM32_RCC + 0x5CU)
#define RCC_APB2ENR REG32(STM32_RCC + 0x60U)
#define RCC_CCIPR REG32(STM32_RCC + 0x88U)
#define RCC_CRRCR REG32(STM32_RCC + 0x98U)

#define RCC_CR_HSION (1U << 8)
#define RCC_CR_HSIRDY (1U << 10)
#define RCC_CR_HSEON (1U << 16)
#define RCC_CR_HSERDY (1U << 17)
#define RCC_CR_HSEBYP (1U << 18)
#define RCC_CRRCR_HSI48ON (1U << 0)
#define RCC_CRRCR_HSI48RDY (1U << 1)
#define RCC_CCIPR_FDCANSEL_POS 24U
#define RCC_CCIPR_CLK48SEL_POS 26U
#define RCC_APB1ENR1_PWREN (1U << 28)

#define STM32_PWR 0x40007000U
#define PWR_CR3 REG32(STM32_PWR + 0x08U)
#define PWR_CR3_UCPD_DBDIS (1U << 14)

#define STM32_FLASH 0x40022000U
#define FLASH_ACR REG32(STM32_FLASH + 0x00U)
#define FLASH_KEYR REG32(STM32_FLASH + 0x08U)
#define FLASH_SR REG32(STM32_FLASH + 0x10U)
#define FLASH_CR REG32(STM32_FLASH + 0x14U)
#define FLASH_ECCR REG32(STM32_FLASH + 0x18U)
#define FLASH_OPTR REG32(STM32_FLASH + 0x20U)

#define FLASH_ACR_PRFTEN (1U << 8)
#define FLASH_ACR_ICEN (1U << 9)
#define FLASH_ACR_DCEN (1U << 10)
#define FLASH_ACR_ICRST (1U << 11)
#define FLASH_ACR_DCRST (1U << 12)
#define FLASH_SR_EOP (1U << 0)
#define FLASH_SR_BSY (1U << 16)
/* OPERR(1), PROGERR(3), WRPERR(4), PGAERR(5), SIZERR(6), PGSERR(7), MISERR(8), FASTERR(9), RDERR(14) */
#define FLASH_SR_ERRORS 0x000043FAU
#define FLASH_CR_PG (1U << 0)
#define FLASH_CR_PER (1U << 1)
#define FLASH_CR_PNB_POS 3U
#define FLASH_CR_BKER (1U << 11)
#define FLASH_CR_STRT (1U << 16)
#define FLASH_CR_LOCK (1U << 31)
#define FLASH_ECCR_ECCD (1U << 31)
#define FLASH_OPTR_IWDG_SW (1U << 16)
#define FLASH_OPTR_DBANK (1U << 22)

#ifndef MUBL_FLASH_PAGE
#define MUBL_FLASH_PAGE 2048U            /* 2 KiB on Cat.2 (G431) and dual-bank Cat.3 (G474) */
#endif
#ifndef MUBL_G4_BANK_SIZE
#define MUBL_G4_BANK_SIZE 0U             /* 0: single-bank; e.g. 256*1024 for dual-bank G474xE */
#endif

#define STM32_CRC 0x40023000U
#define CRC_CLOCK_BIT (1U << 12)
#define CRC_CLOCK_ENR RCC_AHB1ENR
#define STM32_IWDG 0x40003000U
#define STM32_IWDG_HARDWARE() ((FLASH_OPTR & FLASH_OPTR_IWDG_SW) == 0U)
#define STM32_IDCODE 0xE0042000U
#define STM32_UID 0x1FFF7590U

#define GPIO_BASE(port) (0x48000000U + 0x400U * (port))
#define STM32_GPIO_HIGH_SPEED 2U         /* 10 high (11 very high) */
#define GPIO_ENABLE(port) (RCC_AHB2ENR |= 1U << (port))
#define GPIO_RESET(port) (RCC_AHB2RSTR |= 1U << (port), RCC_AHB2RSTR &= ~(1U << (port)))

/* FDCAN1..3 (RM0440 §44): fixed 212-word (848 B) message RAM slice per instance. */
#define FDCAN1_BASE 0x40006400U
#define FDCAN2_BASE 0x40006800U
#define FDCAN3_BASE 0x40006C00U
#define FDCAN_RAM 0x4000A400U
#define FDCAN_CKDIV REG32(FDCAN1_BASE + 0x100U)
#define FDCAN_ENR RCC_APB1ENR1
#define FDCAN_RSTR RCC_APB1RSTR1
#define FDCAN_BIT (1U << 25)

#define USART1_BASE 0x40013800U
#define USART1_ENR RCC_APB2ENR
#define USART1_RSTR RCC_APB2RSTR
#define USART1_BIT (1U << 14)
#define USART1_CLK stm32_pclk2
#define USART2_BASE 0x40004400U
#define USART2_ENR RCC_APB1ENR1
#define USART2_RSTR RCC_APB1RSTR1
#define USART2_BIT (1U << 17)
#define USART2_CLK stm32_pclk1
#define USART3_BASE 0x40004800U
#define USART3_ENR RCC_APB1ENR1
#define USART3_RSTR RCC_APB1RSTR1
#define USART3_BIT (1U << 18)
#define USART3_CLK stm32_pclk1
#define USART4_BASE 0x40004C00U
#define USART4_ENR RCC_APB1ENR1
#define USART4_RSTR RCC_APB1RSTR1
#define USART4_BIT (1U << 19)
#define USART4_CLK stm32_pclk1
#define USART5_BASE 0x40005000U
#define USART5_ENR RCC_APB1ENR1
#define USART5_RSTR RCC_APB1RSTR1
#define USART5_BIT (1U << 20)
#define USART5_CLK stm32_pclk1

/* USB FS device (RM0440 §45): 1 KiB dedicated packet memory at 16-bit stride, internal D+ pull-up
 * (USB_BCDR_DPPU), HSI48 trimmed by CRS against USB SOF.  PA11/PA12 are dedicated analog USB pads
 * when enabled, so MUBL_USB_DM_PIN / MUBL_USB_DP_PIN are left unset (as on F0). */
#define STM32_USB 0x40005C00U
#define STM32_USB_PMA 0x40006000U
#define STM32_USB_PMA_STRIDE 1U
#define STM32_USB_DPPU 1
#define USB_CLOCK_ENR RCC_APB1ENR1
#define USB_RSTR RCC_APB1RSTR1
#define USB_CLOCK_BIT (1U << 23)
#define STM32_CRS 0x40002000U
#define CRS_CR REG32(STM32_CRS + 0x00U)
#define CRS_CFGR REG32(STM32_CRS + 0x04U)
#define CRS_CR_CEN (1U << 5)
#define CRS_CR_AUTOTRIMEN (1U << 6)
#define CRS_CFGR_SYNCSRC_POS 28U
#define CRS_SYNC_USB 2U                  /* 10: USB SOF */
#define CRS_BIT (1U << 8)                /* APB1ENR1, APB1RSTR1 */

extern uint32_t stm32_sysclk, stm32_pclk1, stm32_pclk2;

#endif
