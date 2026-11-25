/* STM32H7 (RM0433 H742/H743/H750/H753, RM0468 H723/H725/H730/H733/H735).
 *
 * Clocks are deliberately simple: the CPU and buses stay on HSI (64 MHz, all prescalers 1, as after
 * reset), the USARTs use their default kernel clock (PCLK), and FDCAN uses HSE directly (its reset
 * kernel clock selection).  No PLL, no voltage scaling change.
 *
 * The supply configuration (PWR_CR3) can be written only once per power-on, and a value that does
 * not match the board's wiring (LDO or SMPS) can stop the part; the bootloader writes the board's
 * MUBL_H7_PWR_CR3 so the application must use the same value. */
#ifndef MUBL_FAMILY_H7_H
#define MUBL_FAMILY_H7_H

#define STM32_RCC 0x58024400U
#define RCC_CR REG32(STM32_RCC + 0x000U)
#define RCC_CFGR REG32(STM32_RCC + 0x010U)
#define RCC_D2CCIP1R REG32(STM32_RCC + 0x050U)
#define RCC_D2CCIP2R REG32(STM32_RCC + 0x054U)
#define RCC_AHB1RSTR REG32(STM32_RCC + 0x080U)
#define RCC_AHB4RSTR REG32(STM32_RCC + 0x088U)
#define RCC_APB1LRSTR REG32(STM32_RCC + 0x090U)
#define RCC_APB1HRSTR REG32(STM32_RCC + 0x094U)
#define RCC_APB2RSTR REG32(STM32_RCC + 0x098U)
#define RCC_AHB1ENR REG32(STM32_RCC + 0x0D8U)
#define RCC_AHB2ENR REG32(STM32_RCC + 0x0DCU)
#define RCC_AHB4ENR REG32(STM32_RCC + 0x0E0U)
#define RCC_APB1LENR REG32(STM32_RCC + 0x0E8U)
#define RCC_APB1HENR REG32(STM32_RCC + 0x0ECU)
#define RCC_APB2ENR REG32(STM32_RCC + 0x0F0U)

#define RCC_CR_HSI48ON (1U << 12)
#define RCC_CR_HSI48RDY (1U << 13)
#define RCC_CR_HSEON (1U << 16)
#define RCC_CR_HSERDY (1U << 17)
#define RCC_CR_HSEBYP (1U << 18)
#define RCC_D2CCIP1R_FDCANSEL_POS 28U
#define RCC_D2CCIP2R_USBSEL (3U << 20)          /* 3: HSI48 */

/* D2-domain SRAM clocks, off after reset.  The application starts with them on, as ArduPilot's
 * bootloader leaves them: ChibiOS's start-up uses its stack there (ArduPilot: SRAM1) before its own
 * clock set-up turns them on.  SRAM3 is on H74x/H75x only (MUBL_H7_SRAM3). */
#define RCC_AHB2ENR_SRAM1EN (1U << 29)
#define RCC_AHB2ENR_SRAM2EN (1U << 30)
#define RCC_AHB2ENR_SRAM3EN (1U << 31)
#ifndef MUBL_H7_SRAM3
#define MUBL_H7_SRAM3 0
#endif
#define STM32_D2_SRAMS (RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | (MUBL_H7_SRAM3 ? RCC_AHB2ENR_SRAM3EN : 0U))

#define STM32_PWR 0x58024800U
#define PWR_CSR1 REG32(STM32_PWR + 0x004U)
#define PWR_CR3 REG32(STM32_PWR + 0x00CU)
#define PWR_CSR1_ACTVOSRDY (1U << 13)
#define PWR_CR3_USB33DEN (1U << 24)
#define PWR_CR3_USB33RDY (1U << 26)

#define STM32_FLASH 0x52002000U
#define FLASH_ACR REG32(STM32_FLASH + 0x000U)
#define FLASH_KEYR(bank) REG32(STM32_FLASH + 0x004U + 0x100U * (bank))
#define FLASH_CR(bank) REG32(STM32_FLASH + 0x00CU + 0x100U * (bank))
#define FLASH_SR(bank) REG32(STM32_FLASH + 0x010U + 0x100U * (bank))
#define FLASH_CCR(bank) REG32(STM32_FLASH + 0x014U + 0x100U * (bank))
#define FLASH_ACR_LATENCY_POS 0U
#define FLASH_ACR_WRHIGHFREQ_POS 4U
#define FLASH_CR_LOCK (1U << 0)
#define FLASH_CR_PG (1U << 1)
#define FLASH_CR_SER (1U << 2)
#define FLASH_CR_PSIZE_POS 4U
#define FLASH_CR_START (1U << 7)
#define FLASH_CR_SNB_POS 8U
#define FLASH_SR_BSY (1U << 0)
#define FLASH_SR_WBNE (1U << 1)
#define FLASH_SR_QW (1U << 2)
#define FLASH_SR_PROG_ERRORS 0x006E0000U   /* WRPERR PGSERR STRBERR INCERR OPERR */
#define FLASH_SR_DBECCERR (1U << 26)
#define FLASH_CCR_ALL 0x0FEF0000U          /* every clearable flag, EOP to CRCEND */
#define FLASH_SECTOR 0x20000U
#define FLASH_BANK_SIZE 0x100000U
#ifndef MUBL_H7_FLASH_PSIZE
#define MUBL_H7_FLASH_PSIZE 2U        /* x32; 0xFF leaves PSIZE alone (RM0455 H7A3/B3 have none) */
#endif
#ifndef MUBL_H7_PWR_CR3
#error "H7 boards must set MUBL_H7_PWR_CR3 to the supply configuration the board is wired for"
#endif

#define STM32_CRC 0x58024C00U
#define CRC_CLOCK_BIT (1U << 19)
#define CRC_CLOCK_ENR RCC_AHB4ENR
#define STM32_IWDG 0x58004800U
#define FLASH_OPTSR_CUR REG32(STM32_FLASH + 0x01CU)
#define FLASH_OPTSR_IWDG1_SW (1U << 4)
#define STM32_IWDG_HARDWARE() ((FLASH_OPTSR_CUR & FLASH_OPTSR_IWDG1_SW) == 0U)   /* option: IWDG1 always on */
#define STM32_IDCODE 0x5C001000U
#define STM32_UID 0x1FF1E800U

#define GPIO_BASE(port) (0x58020000U + 0x400U * (port))
#define STM32_GPIO_HIGH_SPEED 2U       /* 10 high (11 very high) */
#define GPIO_ENABLE(port) (RCC_AHB4ENR |= 1U << (port))
#define GPIO_RESET(port) (RCC_AHB4RSTR |= 1U << (port), RCC_AHB4RSTR &= ~(1U << (port)))

#define FDCAN1_BASE 0x4000A000U
#define FDCAN2_BASE 0x4000A400U
#define FDCAN_RAM 0x4000AC00U
#define FDCAN_ENR RCC_APB1HENR
#define FDCAN_RSTR RCC_APB1HRSTR
#define FDCAN_BIT (1U << 8)

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
#define USART2_ENR RCC_APB1LENR
#define USART2_RSTR RCC_APB1LRSTR
#define USART2_BIT (1U << 17)
#define USART2_CLK stm32_pclk1
#define USART3_BASE 0x40004800U
#define USART3_ENR RCC_APB1LENR
#define USART3_RSTR RCC_APB1LRSTR
#define USART3_BIT (1U << 18)
#define USART3_CLK stm32_pclk1
#define USART4_BASE 0x40004C00U
#define USART4_ENR RCC_APB1LENR
#define USART4_RSTR RCC_APB1LRSTR
#define USART4_BIT (1U << 19)
#define USART4_CLK stm32_pclk1
#define USART5_BASE 0x40005000U
#define USART5_ENR RCC_APB1LENR
#define USART5_RSTR RCC_APB1LRSTR
#define USART5_BIT (1U << 20)
#define USART5_CLK stm32_pclk1
#define USART7_BASE 0x40007800U
#define USART7_ENR RCC_APB1LENR
#define USART7_RSTR RCC_APB1LRSTR
#define USART7_BIT (1U << 30)
#define USART7_CLK stm32_pclk1
#define USART8_BASE 0x40007C00U
#define USART8_ENR RCC_APB1LENR
#define USART8_RSTR RCC_APB1LRSTR
#define USART8_BIT (1U << 31)
#define USART8_CLK stm32_pclk1

/* USB: an OTG core with its embedded full-speed PHY, chosen by MUBL_USB_OTG_INSTANCE: 1 OTG_HS
 * (USB1: H74x/H75x PB14/PB15, H72x/H73x PA11/PA12), 2 OTG_FS (USB2, H74x/H75x PA11/PA12).  48 MHz
 * from HSI48, trimmed by the CRS against that core's start-of-frame packets; VDD33USB must be
 * supplied.  VBUS sensing off (B-session override). */
#ifndef MUBL_USB_OTG_INSTANCE
#define MUBL_USB_OTG_INSTANCE 1
#endif
#if MUBL_USB_OTG_INSTANCE == 1
#define STM32_OTG 0x40040000U
#define OTG_CLOCK_BIT (1U << 25)
#define CRS_SYNC_USB 2U
#else
#define STM32_OTG 0x40080000U
#define OTG_CLOCK_BIT (1U << 27)
#define CRS_SYNC_USB 3U
#endif
#define OTG_CLOCK_ENR RCC_AHB1ENR
#define OTG_RSTR RCC_AHB1RSTR
#define STM32_OTG_VBDEN 1
#define STM32_CRS 0x40008400U
#define CRS_CR REG32(STM32_CRS + 0x00U)
#define CRS_CFGR REG32(STM32_CRS + 0x04U)
#define CRS_CR_CEN (1U << 5)
#define CRS_CR_AUTOTRIMEN (1U << 6)
#define CRS_CFGR_SYNCSRC_POS 28U
#define CRS_BIT (1U << 1)                    /* APB1HENR, APB1HRSTR */

extern uint32_t stm32_sysclk, stm32_pclk1, stm32_pclk2;

#endif
