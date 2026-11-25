/* STM32F4 (RM0090, RM0390, RM0368).  Clock: MUBL_SYSCLK_HZ (168 MHz) from the PLL, from HSE if
 * there is one.  The voltage scale is left at its reset value, so MUBL_SYSCLK_HZ must be within
 * what that allows on the part (168 MHz on F405/F407/F446 without overdrive, 84 MHz on F401). */
#ifndef MUBL_FAMILY_F4_H
#define MUBL_FAMILY_F4_H

#define STM32_USART_V1 1

#define STM32_RCC 0x40023800U
#define RCC_CR REG32(STM32_RCC + 0x00U)
#define RCC_PLLCFGR REG32(STM32_RCC + 0x04U)
#define RCC_CFGR REG32(STM32_RCC + 0x08U)
#define RCC_AHB1RSTR REG32(STM32_RCC + 0x10U)
#define RCC_AHB2RSTR REG32(STM32_RCC + 0x14U)
#define RCC_APB1RSTR REG32(STM32_RCC + 0x20U)
#define RCC_APB2RSTR REG32(STM32_RCC + 0x24U)
#define RCC_AHB1ENR REG32(STM32_RCC + 0x30U)
#define RCC_AHB2ENR REG32(STM32_RCC + 0x34U)
#define RCC_APB1ENR REG32(STM32_RCC + 0x40U)
#define RCC_APB2ENR REG32(STM32_RCC + 0x44U)

#define RCC_CR_HSION (1U << 0)
#define RCC_CR_HSIRDY (1U << 1)
#define RCC_CR_HSEON (1U << 16)
#define RCC_CR_HSERDY (1U << 17)
#define RCC_CR_HSEBYP (1U << 18)
#define RCC_CR_PLLON (1U << 24)
#define RCC_CR_PLLRDY (1U << 25)
#define RCC_PLLCFGR_PLLN_POS 6U
#define RCC_PLLCFGR_PLLP_POS 16U
#define RCC_PLLCFGR_PLLSRC_HSE (1U << 22)
#define RCC_PLLCFGR_PLLQ_POS 24U
#define RCC_PLLCFGR_RESET 0x24003010U
#define RCC_CFGR_SW_PLL 2U
#define RCC_CFGR_SWS_POS 2U
#define RCC_CFGR_PPRE1_POS 10U
#define RCC_CFGR_PPRE2_POS 13U

#define STM32_FLASH 0x40023C00U
#define FLASH_ACR REG32(STM32_FLASH + 0x00U)
#define FLASH_KEYR REG32(STM32_FLASH + 0x04U)
#define FLASH_SR REG32(STM32_FLASH + 0x0CU)
#define FLASH_CR REG32(STM32_FLASH + 0x10U)
#define FLASH_ACR_PRFTEN (1U << 8)
#define FLASH_ACR_ICEN (1U << 9)
#define FLASH_ACR_DCEN (1U << 10)
#define FLASH_ACR_ICRST (1U << 11)
#define FLASH_ACR_DCRST (1U << 12)
#define FLASH_SR_ERRORS 0xF2U        /* PGSERR PGPERR PGAERR WRPERR OPERR */
#define FLASH_SR_BSY (1U << 16)
#define FLASH_CR_PG (1U << 0)
#define FLASH_CR_SER (1U << 1)
#define FLASH_CR_SNB_POS 3U
#define FLASH_CR_PSIZE_POS 8U
#define FLASH_CR_STRT (1U << 16)
#define FLASH_CR_LOCK (1U << 31)
#ifndef MUBL_F4_PSIZE
#define MUBL_F4_PSIZE 2U             /* x32: VDD 2.7-3.6 V (the granule is 4 bytes) */
#endif

#define STM32_IWDG 0x40003000U
#define FLASH_OPTCR REG32(STM32_FLASH + 0x14U)
#define FLASH_OPTCR_WDG_SW (1U << 5)
#define STM32_IWDG_HARDWARE() ((FLASH_OPTCR & FLASH_OPTCR_WDG_SW) == 0U)   /* option: IWDG always on */
#define STM32_IDCODE 0xE0042000U
#define STM32_UID 0x1FFF7A10U

#define GPIO_BASE(port) (0x40020000U + 0x400U * (port))
#define STM32_GPIO_HIGH_SPEED 2U       /* 10 high (11 very high) */
#define GPIO_ENABLE(port) (RCC_AHB1ENR |= 1U << (port))
#define GPIO_RESET(port) (RCC_AHB1RSTR |= 1U << (port), RCC_AHB1RSTR &= ~(1U << (port)))

/* CAN2's filters live in CAN1, which must be clocked; CAN2 gets banks from 14 (CAN2SB reset). */
#define CAN1_BASE 0x40006400U
#define CAN1_ENR RCC_APB1ENR
#define CAN1_RSTR RCC_APB1RSTR
#define CAN1_BIT (1U << 25)
#define CAN1_CLK stm32_pclk1
#define CAN1_FILTER_BASE CAN1_BASE
#define CAN1_FIRST_BANK 0U
#define CAN2_BASE 0x40006800U
#define CAN2_ENR RCC_APB1ENR
#define CAN2_RSTR RCC_APB1RSTR
#define CAN2_BIT ((1U << 26) | (1U << 25))
#define CAN2_CLK stm32_pclk1
#define CAN2_FILTER_BASE CAN1_BASE
#define CAN2_FIRST_BANK 14U

#define USART1_BASE 0x40011000U
#define USART1_ENR RCC_APB2ENR
#define USART1_RSTR RCC_APB2RSTR
#define USART1_BIT (1U << 4)
#define USART1_CLK stm32_pclk2
#define USART6_BASE 0x40011400U
#define USART6_ENR RCC_APB2ENR
#define USART6_RSTR RCC_APB2RSTR
#define USART6_BIT (1U << 5)
#define USART6_CLK stm32_pclk2
#define USART2_BASE 0x40004400U
#define USART2_ENR RCC_APB1ENR
#define USART2_RSTR RCC_APB1RSTR
#define USART2_BIT (1U << 17)
#define USART2_CLK stm32_pclk1
#define USART3_BASE 0x40004800U
#define USART3_ENR RCC_APB1ENR
#define USART3_RSTR RCC_APB1RSTR
#define USART3_BIT (1U << 18)
#define USART3_CLK stm32_pclk1
#define USART4_BASE 0x40004C00U
#define USART4_ENR RCC_APB1ENR
#define USART4_RSTR RCC_APB1RSTR
#define USART4_BIT (1U << 19)
#define USART4_CLK stm32_pclk1
#define USART5_BASE 0x40005000U
#define USART5_ENR RCC_APB1ENR
#define USART5_RSTR RCC_APB1RSTR
#define USART5_BIT (1U << 20)
#define USART5_CLK stm32_pclk1

/* USB: OTG_FS, 48 MHz from the PLL's Q output (needs the crystal).  VBUS sensing is off: the F405/
 * F407 core takes GCCFG.NOVBUSSENS; F446/F412-class cores (MUBL_F4_OTG_VBDEN) the B-session
 * override instead. */
#define STM32_OTG 0x50000000U
#define OTG_CLOCK_ENR RCC_AHB2ENR
#define OTG_RSTR RCC_AHB2RSTR
#define OTG_CLOCK_BIT (1U << 7)
#ifdef MUBL_F4_OTG_VBDEN
#define STM32_OTG_VBDEN 1
#else
#define STM32_OTG_VBDEN 0
#endif

extern uint32_t stm32_sysclk, stm32_pclk1, stm32_pclk2;

#endif
