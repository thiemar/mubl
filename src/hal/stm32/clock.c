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

static bool on_hse;

void stm32_clock_init(void)
{
    FLASH_ACR = LATENCY | FLASH_ACR_PRFTBE;
    on_hse = start_hse();
    if (on_hse) {
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

#if MUBL_USB
#if MUBL_FAMILY == MUBL_FAMILY_F0
MUBL_STATIC_ASSERT(MUBL_HSE_HZ != 0 || MUBL_F0_HSI48, "USB needs a crystal or HSI48 (MUBL_F0_HSI48)");

/* 48 MHz from the crystal's PLL, or HSI48 kept within USB's tolerance by the CRS, which trims it
 * against the host's start-of-frame packets (its reset configuration). */
bool stm32_usb_clock(bool on)
{
    if (on && on_hse) {
        RCC_CFGR3 |= RCC_CFGR3_USBSW;
    } else if (on) {
        RCC_CFGR3 &= ~RCC_CFGR3_USBSW;
        RCC_APB1ENR |= CRS_BIT;
        CRS_CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
    } else {
        RCC_APB1RSTR |= CRS_BIT;
        RCC_APB1RSTR &= ~CRS_BIT;
        RCC_APB1ENR &= ~CRS_BIT;
        RCC_CFGR3 &= ~RCC_CFGR3_USBSW;
    }
    return true;
}
#else
/* PLL 72 MHz / 1.5 (USBPRE 0, as after reset).  Without the crystal (it did not start) the PLL
 * runs from HSI at 64 MHz: no 48 MHz, so USB stays off the bus. */
MUBL_STATIC_ASSERT(MUBL_HSE_HZ != 0, "USB on F3 needs a crystal");
bool stm32_usb_clock(bool on)
{
    return !on || on_hse;
}
#endif
#endif

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

static bool on_hse;

void stm32_clock_init(void)
{
    const bool hse = start_hse();
    on_hse = hse;
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

#if MUBL_USB
/* The PLL's Q output: exactly 48 MHz only from a VCO that is a multiple of 48 MHz (168 MHz, not
 * 180), and accurate enough only from the crystal. */
MUBL_STATIC_ASSERT(PLL_N % PLL_Q == 0 && PLL_N / PLL_Q == 48, "USB needs a VCO of a multiple of 48 MHz (MUBL_SYSCLK_HZ)");
MUBL_STATIC_ASSERT(MUBL_HSE_HZ != 0, "USB on F4 needs a crystal");
bool stm32_usb_clock(bool on)
{
    return !on || on_hse;
}
#endif

#elif MUBL_FAMILY == MUBL_FAMILY_G4

void stm32_clock_init(void)
{
    /* Disable UCPD dead-battery pull-down resistors on PA9 / PB6 (needed when USART1 uses PA9). */
    RCC_APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC_APB1ENR1;
    PWR_CR3 |= PWR_CR3_UCPD_DBDIS;
    RCC_APB1ENR1 &= ~RCC_APB1ENR1_PWREN;
    (void)start_hse();                              /* FDCAN's kernel clock */
    stm32_sysclk = stm32_pclk1 = stm32_pclk2 = 16000000U;
}

void stm32_clock_deinit(void)
{
    RCC_CR &= ~RCC_CR_HSEON;
    while (RCC_CR & RCC_CR_HSERDY) {
    }
    RCC_CR &= ~RCC_CR_HSEBYP;                       /* writable once HSE is off */
}

#if MUBL_USB
/* HSI48 for the USB FS device peripheral, trimmed by the CRS against USB SOF packets. */
bool stm32_usb_clock(bool on)
{
    if (on) {
        RCC_CRRCR |= RCC_CRRCR_HSI48ON;
        if (!wait_set(&RCC_CRRCR, RCC_CRRCR_HSI48RDY, HSE_TIMEOUT)) return false;
        RCC_CCIPR &= ~(3U << RCC_CCIPR_CLK48SEL_POS);    /* 00: HSI48 */
        RCC_APB1ENR1 |= CRS_BIT;
        CRS_CFGR = (CRS_CFGR & ~(3U << CRS_CFGR_SYNCSRC_POS)) | (CRS_SYNC_USB << CRS_CFGR_SYNCSRC_POS);
        CRS_CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
        return true;
    }
    RCC_APB1RSTR1 |= CRS_BIT;
    RCC_APB1RSTR1 &= ~CRS_BIT;
    RCC_APB1ENR1 &= ~CRS_BIT;
    RCC_CRRCR &= ~RCC_CRRCR_HSI48ON;
    return true;
}
#endif

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

#if MUBL_USB
/* HSI48 for the OTG core, trimmed by the CRS from its start-of-frame packets; false if VDD33USB is
 * not there. */
bool stm32_usb_clock(bool on)
{
    if (on) {
        PWR_CR3 |= PWR_CR3_USB33DEN;
        if (!wait_set(&PWR_CR3, PWR_CR3_USB33RDY, HSE_TIMEOUT)) return false;
        RCC_CR |= RCC_CR_HSI48ON;
        if (!wait_set(&RCC_CR, RCC_CR_HSI48RDY, HSE_TIMEOUT)) return false;
        RCC_D2CCIP2R |= RCC_D2CCIP2R_USBSEL;
        RCC_APB1HENR |= CRS_BIT;
        CRS_CFGR = (CRS_CFGR & ~(3U << CRS_CFGR_SYNCSRC_POS)) | (CRS_SYNC_USB << CRS_CFGR_SYNCSRC_POS);
        CRS_CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
        return true;
    }
    RCC_APB1HRSTR |= CRS_BIT;
    RCC_APB1HRSTR &= ~CRS_BIT;
    RCC_APB1HENR &= ~CRS_BIT;
    RCC_D2CCIP2R &= ~RCC_D2CCIP2R_USBSEL;
    RCC_CR &= ~RCC_CR_HSI48ON;
    PWR_CR3 &= ~PWR_CR3_USB33DEN;
    return true;
}
#endif

#endif
