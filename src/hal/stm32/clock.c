/*
 * System clocks per family, and their return to the reset configuration before the application
 * starts (it may assume the PLL is off when it configures its own).
 *
 * Board settings: MUBL_HSE_HZ (0: no crystal), MUBL_HSE_BYPASS (an external clock rather than a
 * crystal), and on F4 MUBL_SYSCLK_HZ.  If the HSE does not start the bootloader runs from the
 * internal oscillator instead; CAN timing is then only as good as the HSI.
 */
#include "stm32.h"

#ifndef MUBL_HSE_HZ
#define MUBL_HSE_HZ 0
#endif
#ifndef MUBL_HSE_BYPASS
#define MUBL_HSE_BYPASS 0
#endif
#define HSE_TIMEOUT 500000U

uint32_t stm32_sysclk, stm32_pclk1, stm32_pclk2;

static bool wait_set(volatile uint32_t* reg, uint32_t mask, uint32_t timeout)
{
    while ((*reg & mask) != mask) {
        if (timeout-- == 0U) return false;
    }
    return true;
}

static bool start_hse(void)
{
    if (MUBL_HSE_HZ == 0) return false;
    if (MUBL_HSE_BYPASS) RCC_CR |= RCC_CR_HSEBYP;     /* writable only while HSE is off */
    RCC_CR |= RCC_CR_HSEON;
    if (wait_set(&RCC_CR, RCC_CR_HSERDY, HSE_TIMEOUT)) return true;
    RCC_CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
    return false;
}

#if MUBL_FAMILY == MUBL_FAMILY_F0 || MUBL_FAMILY == MUBL_FAMILY_F3

#if MUBL_FAMILY == MUBL_FAMILY_F0
#define SYSCLK 48000000U
#define HSI_PLL_MUL 12U                /* HSI/2 x 12 = 48 MHz */
#define LATENCY 1U
#define APB1_DIV_BITS 0U
#define APB1_SHIFT 0U
#else
#define SYSCLK 72000000U
#define HSI_PLL_MUL 16U                /* HSI/2 x 16 = 64 MHz */
#define LATENCY 2U
#define APB1_DIV_BITS (4U << RCC_CFGR_PPRE1_POS)   /* /2: APB1 at most 36 MHz */
#define APB1_SHIFT 1U
#endif

#if MUBL_HSE_HZ
#define PREDIV ((SYSCLK % MUBL_HSE_HZ) == 0 ? 1U : 2U)
#define PLL_MUL (SYSCLK / (MUBL_HSE_HZ / PREDIV))
MUBL_STATIC_ASSERT(PLL_MUL >= 2 && PLL_MUL <= 16 && PLL_MUL * (MUBL_HSE_HZ / PREDIV) == SYSCLK,
                   "MUBL_HSE_HZ cannot make the system clock");
#else
#define PREDIV 1U
#define PLL_MUL 2U
#endif

static void pll_on(uint32_t cfgr, uint32_t mul)
{
    RCC_CFGR = cfgr | ((mul - 2U) << RCC_CFGR_PLLMUL_POS) | APB1_DIV_BITS;
    RCC_CR |= RCC_CR_PLLON;
    (void)wait_set(&RCC_CR, RCC_CR_PLLRDY, HSE_TIMEOUT);
    RCC_CFGR |= RCC_CFGR_SW_PLL;
    while (((RCC_CFGR >> RCC_CFGR_SWS_POS) & 3U) != RCC_CFGR_SW_PLL) {
    }
}

void stm32_clock_init(void)
{
    FLASH_ACR = LATENCY | FLASH_ACR_PRFTBE;
    if (start_hse()) {
        RCC_CFGR2 = PREDIV - 1U;
        pll_on(RCC_CFGR_PLLSRC_HSE_PREDIV, PLL_MUL);
        stm32_sysclk = SYSCLK;
#if MUBL_FAMILY == MUBL_FAMILY_F0 && MUBL_F0_HSI48
    } else if (RCC_CR2 |= RCC_CR2_HSI48ON, wait_set(&RCC_CR2, RCC_CR2_HSI48RDY, HSE_TIMEOUT)) {
        RCC_CFGR = RCC_CFGR_SW_HSI48;
        while (((RCC_CFGR >> RCC_CFGR_SWS_POS) & 3U) != RCC_CFGR_SW_HSI48) {
        }
        stm32_sysclk = 48000000U;
#endif
    } else {
        pll_on(0, HSI_PLL_MUL);
        stm32_sysclk = 4000000U * HSI_PLL_MUL;
    }
    stm32_pclk1 = stm32_sysclk >> APB1_SHIFT;
    stm32_pclk2 = stm32_sysclk;
}

void stm32_clock_deinit(void)
{
    RCC_CR |= RCC_CR_HSION;
    (void)wait_set(&RCC_CR, RCC_CR_HSIRDY, HSE_TIMEOUT);
    RCC_CFGR = 0;                                   /* SYSCLK = HSI, prescalers 1 */
    while (((RCC_CFGR >> RCC_CFGR_SWS_POS) & 3U) != 0U) {
    }
    RCC_CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);
    while (RCC_CR & (RCC_CR_PLLRDY | RCC_CR_HSERDY)) {
    }
    RCC_CR &= ~RCC_CR_HSEBYP;                       /* writable once HSE is off */
    RCC_CFGR = 0;                                   /* PLLSRC/PLLMUL: writable once the PLL is off */
    RCC_CFGR2 = 0;
#if MUBL_FAMILY == MUBL_FAMILY_F0
    RCC_CR2 &= ~RCC_CR2_HSI48ON;
#endif
    FLASH_ACR = FLASH_ACR_PRFTBE;                   /* zero wait states (reset value) */
}

#elif MUBL_FAMILY == MUBL_FAMILY_F4

#ifndef MUBL_SYSCLK_HZ
#define MUBL_SYSCLK_HZ 168000000U
#endif
#ifndef MUBL_APB1_MAX_HZ
#define MUBL_APB1_MAX_HZ 42000000U
#endif
#ifndef MUBL_APB2_MAX_HZ
#define MUBL_APB2_MAX_HZ 84000000U
#endif
/* VCO input 1 MHz; VCO 192-432 MHz; P the smallest even divider that keeps the VCO in range. */
#define PLL_P (MUBL_SYSCLK_HZ * 2U >= 192000000U ? 2U : MUBL_SYSCLK_HZ * 4U >= 192000000U ? 4U : \
               MUBL_SYSCLK_HZ * 6U >= 192000000U ? 6U : 8U)
#define PLL_N (MUBL_SYSCLK_HZ / 1000000U * PLL_P)
#define PLL_Q ((PLL_N + 47U) / 48U)                  /* 48 MHz or less on the USB/SDIO clock */
#define DIV_FOR(max) (MUBL_SYSCLK_HZ <= (max) ? 1U : MUBL_SYSCLK_HZ / 2U <= (max) ? 2U : \
                      MUBL_SYSCLK_HZ / 4U <= (max) ? 4U : 8U)
#define PPRE(div) ((div) == 1U ? 0U : (div) == 2U ? 4U : (div) == 4U ? 5U : 6U)
#define LATENCY ((MUBL_SYSCLK_HZ - 1U) / 30000000U)  /* VDD 2.7-3.6 V */
MUBL_STATIC_ASSERT(MUBL_SYSCLK_HZ % 1000000U == 0 && PLL_N >= 50U && PLL_N <= 432U && PLL_Q <= 15U,
                   "MUBL_SYSCLK_HZ cannot be made by the PLL");
MUBL_STATIC_ASSERT(MUBL_HSE_HZ % 1000000 == 0, "MUBL_HSE_HZ must be a whole number of MHz");

void stm32_clock_init(void)
{
    const bool hse = start_hse();
    const uint32_t m = hse ? MUBL_HSE_HZ / 1000000U : 16U;
    FLASH_ACR = LATENCY | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC_CFGR = (PPRE(DIV_FOR(MUBL_APB1_MAX_HZ)) << RCC_CFGR_PPRE1_POS) |
               (PPRE(DIV_FOR(MUBL_APB2_MAX_HZ)) << RCC_CFGR_PPRE2_POS);
    RCC_PLLCFGR = m | (PLL_N << RCC_PLLCFGR_PLLN_POS) | ((PLL_P / 2U - 1U) << RCC_PLLCFGR_PLLP_POS) |
                  (PLL_Q << RCC_PLLCFGR_PLLQ_POS) | (hse ? RCC_PLLCFGR_PLLSRC_HSE : 0U) |
                  (RCC_PLLCFGR_RESET & 0x70000000U);          /* PLLR (F446) as after reset */
    RCC_CR |= RCC_CR_PLLON;
    (void)wait_set(&RCC_CR, RCC_CR_PLLRDY, HSE_TIMEOUT);
    RCC_CFGR |= RCC_CFGR_SW_PLL;
    while (((RCC_CFGR >> RCC_CFGR_SWS_POS) & 3U) != RCC_CFGR_SW_PLL) {
    }
    stm32_sysclk = MUBL_SYSCLK_HZ;
    stm32_pclk1 = MUBL_SYSCLK_HZ / DIV_FOR(MUBL_APB1_MAX_HZ);
    stm32_pclk2 = MUBL_SYSCLK_HZ / DIV_FOR(MUBL_APB2_MAX_HZ);
}

void stm32_clock_deinit(void)
{
    RCC_CR |= RCC_CR_HSION;
    (void)wait_set(&RCC_CR, RCC_CR_HSIRDY, HSE_TIMEOUT);
    RCC_CFGR = 0;
    while (((RCC_CFGR >> RCC_CFGR_SWS_POS) & 3U) != 0U) {
    }
    RCC_CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);
    while (RCC_CR & (RCC_CR_PLLRDY | RCC_CR_HSERDY)) {
    }
    RCC_CR &= ~RCC_CR_HSEBYP;                       /* writable once HSE is off */
    RCC_PLLCFGR = RCC_PLLCFGR_RESET;                /* writable once the PLL is off */
    FLASH_ACR = 0;                                  /* caches off, then flushed, then reset value */
    FLASH_ACR = FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH_ACR = 0;
}

#elif MUBL_FAMILY == MUBL_FAMILY_H7

void stm32_clock_init(void)
{
    /* Write the supply configuration (once per power-on) and wait for the voltage to settle. */
    PWR_CR3 = MUBL_H7_PWR_CR3;
    (void)wait_set(&PWR_CSR1, PWR_CSR1_ACTVOSRDY, HSE_TIMEOUT);
    /* 64 MHz at VOS3: 2 wait states and programming delay 1 suit every RM0433/RM0468 part. */
    FLASH_ACR = (2U << FLASH_ACR_LATENCY_POS) | (1U << FLASH_ACR_WRHIGHFREQ_POS);
    (void)start_hse();                              /* FDCAN's kernel clock */
    stm32_sysclk = stm32_pclk1 = stm32_pclk2 = 64000000U;
}

void stm32_clock_deinit(void)
{
    RCC_CR &= ~RCC_CR_HSEON;
    while (RCC_CR & RCC_CR_HSERDY) {
    }
    RCC_CR &= ~RCC_CR_HSEBYP;                       /* writable once HSE is off */
}

#endif
