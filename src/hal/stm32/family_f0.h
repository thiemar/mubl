/* STM32F0 (RM0091).  Clock: 48 MHz from HSE via the PLL, else HSI48 (F04x/F07x/F09x), else HSI/2. */
#ifndef MUBL_FAMILY_F0_H
#define MUBL_FAMILY_F0_H

#define STM32_CORTEX_M0 1
#ifndef MUBL_F0_HSI48
#define MUBL_F0_HSI48 MUBL_USB /* 1: without HSE run from HSI48 (F04x, F07x, F09x) instead of HSI/2 */
#endif

#define STM32_RCC 0x40021000U
#define RCC_CR REG32(STM32_RCC + 0x00U)
#define RCC_CFGR REG32(STM32_RCC + 0x04U)
#define RCC_APB2RSTR REG32(STM32_RCC + 0x0CU)
#define RCC_APB1RSTR REG32(STM32_RCC + 0x10U)
#define RCC_AHBENR REG32(STM32_RCC + 0x14U)
#define RCC_APB2ENR REG32(STM32_RCC + 0x18U)
#define RCC_APB1ENR REG32(STM32_RCC + 0x1CU)
#define RCC_AHBRSTR REG32(STM32_RCC + 0x28U)
#define RCC_CFGR2 REG32(STM32_RCC + 0x2CU)
#define RCC_CFGR3 REG32(STM32_RCC + 0x30U)
#define RCC_CR2 REG32(STM32_RCC + 0x34U)

#define RCC_CR_HSION (1U << 0)
#define RCC_CR_HSIRDY (1U << 1)
#define RCC_CR_HSEON (1U << 16)
#define RCC_CR_HSERDY (1U << 17)
#define RCC_CR_HSEBYP (1U << 18)
#define RCC_CR_PLLON (1U << 24)
#define RCC_CR_PLLRDY (1U << 25)
#define RCC_CR2_HSI48ON (1U << 16)
#define RCC_CR2_HSI48RDY (1U << 17)
#define RCC_CFGR_SW_PLL 2U
#define RCC_CFGR_SW_HSI48 3U
#define RCC_CFGR_SWS_POS 2U
#define RCC_CFGR_PLLSRC_HSE_PREDIV (1U << 16)
#define RCC_CFGR_PLLMUL_POS 18U
#define RCC_APB2ENR_SYSCFGEN (1U << 0)
#define RCC_CFGR3_USBSW (1U << 7)      /* USB clock: 0 HSI48, 1 PLL */

#define STM32_FLASH 0x40022000U
#define FLASH_ACR REG32(STM32_FLASH + 0x00U)
#define FLASH_KEYR REG32(STM32_FLASH + 0x04U)
#define FLASH_SR REG32(STM32_FLASH + 0x0CU)
#define FLASH_CR REG32(STM32_FLASH + 0x10U)
#define FLASH_AR REG32(STM32_FLASH + 0x14U)
#define FLASH_ACR_PRFTBE (1U << 4)
#define FLASH_SR_BSY (1U << 0)
#define FLASH_SR_PGERR (1U << 2)
#define FLASH_SR_WRPRTERR (1U << 4)
#define FLASH_SR_EOP (1U << 5)
#define FLASH_CR_PG (1U << 0)
#define FLASH_CR_PER (1U << 1)
#define FLASH_CR_STRT (1U << 6)
#define FLASH_CR_LOCK (1U << 7)
#ifndef MUBL_FLASH_PAGE
#define MUBL_FLASH_PAGE 1024U        /* 2048 on F07x/F09x */
#endif

#define STM32_SYSCFG 0x40010000U
#define SYSCFG_CFGR1 REG32(STM32_SYSCFG + 0x00U)
#define SYSCFG_CFGR1_MEM_MODE 3U
#define SYSCFG_CFGR1_PA11_PA12_RMP (1U << 4)

#define STM32_CRC 0x40023000U
#define CRC_CLOCK_BIT (1U << 6)
#define CRC_CLOCK_ENR RCC_AHBENR
#define STM32_IWDG 0x40003000U
#define FLASH_OBR REG32(STM32_FLASH + 0x1CU)
#define FLASH_OBR_IWDG_SW (1U << 8)
#define STM32_IWDG_HARDWARE() ((FLASH_OBR & FLASH_OBR_IWDG_SW) == 0U)   /* option: IWDG always on */
#define STM32_IDCODE 0x40015800U
#define STM32_UID 0x1FFFF7ACU

#define GPIO_BASE(port) (0x48000000U + 0x400U * (port))
#define STM32_GPIO_HIGH_SPEED 3U       /* x0 low, 01 medium, 11 high */
#define GPIO_ENABLE(port) (RCC_AHBENR |= 1U << (17U + (port)))
#define GPIO_RESET(port) (RCC_AHBRSTR |= 1U << (17U + (port)), RCC_AHBRSTR &= ~(1U << (17U + (port))))

#define CAN1_BASE 0x40006400U
#define CAN1_ENR RCC_APB1ENR
#define CAN1_RSTR RCC_APB1RSTR
#define CAN1_BIT (1U << 25)
#define CAN1_CLK stm32_pclk1
#define CAN1_FILTER_BASE CAN1_BASE
#define CAN1_FIRST_BANK 0U

#define USART1_BASE 0x40013800U
#define USART1_ENR RCC_APB2ENR
#define USART1_RSTR RCC_APB2RSTR
#define USART1_BIT (1U << 14)
#define USART1_CLK stm32_pclk1
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

/* USB (F04x, F07x, F09x): 1 KiB packet memory at 16-bit stride, internal D+ pull-up; HSI48
 * trimmed by the CRS against the host's start-of-frame packets when there is no crystal. */
#define STM32_USB 0x40005C00U
#define STM32_USB_PMA 0x40006000U
#define STM32_USB_PMA_STRIDE 1U
#define STM32_USB_DPPU 1
#define USB_CLOCK_ENR RCC_APB1ENR
#define USB_RSTR RCC_APB1RSTR
#define USB_CLOCK_BIT (1U << 23)
#define STM32_CRS 0x40006C00U
#define CRS_CR REG32(STM32_CRS + 0x00U)
#define CRS_CR_CEN (1U << 5)
#define CRS_CR_AUTOTRIMEN (1U << 6)
#define CRS_BIT (1U << 27)

extern uint32_t stm32_sysclk, stm32_pclk1, stm32_pclk2;

#endif
