/*
 * Start-up, time base, watchdog, fault handling and the jump to the application.
 */
#include "hal/hal.h"
#include "stm32.h"

extern void _estack(void);           /* linker symbols; _estack declared as a function for the table */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);

static volatile uint32_t ticks;

/* ---- Vectors and start-up --------------------------------------------------------------- */

void Reset_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void Default_Handler(void);

__attribute__((section(".isr_vector"), used)) static void (*const vectors[16])(void) = {
    _estack, Reset_Handler,
#if MUBL_FAMILY == MUBL_FAMILY_G4
    HardFault_Handler,   /* STM32G4 raises NMI on a double-bit flash ECC error */
#else
    Default_Handler,
#endif
    HardFault_Handler,
    Default_Handler, HardFault_Handler, Default_Handler, 0, 0, 0, 0,
    Default_Handler, Default_Handler, 0, Default_Handler, SysTick_Handler,
};

/* Interrupts as after reset: SysTick off, every NVIC source disabled and nothing pending.  ARMv6-M
 * has one ICER/ICPR register, ARMv7-M up to eight. */
static void quiesce(void)
{
    SYST_CSR = 0;
    SYST_RVR = 0;
    SYST_CVR = 0;
    for (uint32_t i = 0; i < (STM32_CORTEX_M0 ? 1U : 8U); i++) {
        NVIC_ICER(i) = 0xFFFFFFFFU;
        NVIC_ICPR(i) = 0xFFFFFFFFU;
    }
    SCB_ICSR = ICSR_PENDSTCLR | ICSR_PENDSVCLR;
}

__attribute__((noreturn)) void Reset_Handler(void)
{
    uint32_t* s = &_sidata;
    for (uint32_t* d = &_sdata; d < &_edata;) *d++ = *s++;
    for (uint32_t* d = &_sbss; d < &_ebss;) *d++ = 0;
    /* An application may jump here without a reset: take back the vectors and interrupts.  The
     * table has only the core's 16 entries, so no external interrupt may stay enabled. */
    quiesce();
#if STM32_CORTEX_M0
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG_CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;
#else
    SCB_VTOR = MUBL_FLASH_BASE;
#endif
    __asm volatile("cpsie i" ::: "memory");
    main();
    for (;;) {
    }
}

void SysTick_Handler(void)
{
    ticks++;
}

void Default_Handler(void)
{
    hal_reset();
}

/* ---- Bus faults on flash reads (ECC) ------------------------------------------------------ */

#if MUBL_FLASH_ECC
/* hal_guarded saves the callee-saved registers and its stack pointer, then calls fn.  A fault
 * while guard_sp is set resumes at mubl_guard_fail, which restores that stack and returns false
 * from hal_guarded.  Ten registers keep the stack 8-byte aligned for the call. */
__attribute__((used)) volatile uint32_t mubl_guard_sp;   /* also referenced from asm, which LTO cannot see */

__attribute__((naked)) bool hal_guarded(__attribute__((unused)) void (*fn)(void*), __attribute__((unused)) void* arg)
{
    __asm volatile(
        "push {r3-r11, lr}\n"
        "ldr r2, =mubl_guard_sp\n"
        "mov r3, sp\n"
        "str r3, [r2]\n"
        "mov r3, r0\n"
        "mov r0, r1\n"
        "blx r3\n"
        "ldr r2, =mubl_guard_sp\n"
        "movs r3, #0\n"
        "str r3, [r2]\n"
        "movs r0, #1\n"
        "pop {r3-r11, pc}\n"
        ".global mubl_guard_fail\n"
        "mubl_guard_fail:\n"
        "ldr r2, =mubl_guard_sp\n"
        "ldr r3, [r2]\n"
        "mov sp, r3\n"
        "movs r3, #0\n"
        "str r3, [r2]\n"
        "movs r0, #0\n"
        "pop {r3-r11, pc}\n"
        ".ltorg\n");
}

__attribute__((used)) void mubl_fault(uint32_t* frame)
{
    extern char mubl_guard_fail[];
    if (mubl_guard_sp != 0U) {
        frame[6] = (uint32_t)(uintptr_t)mubl_guard_fail & ~1U;    /* stacked PC */
        /* Thumb, outside any IT block, thread mode; keep bit 9 (the frame's stack realignment). */
        frame[7] = (frame[7] & (1U << 9)) | (1U << 24);
        SCB_CFSR = SCB_CFSR;                                        /* write-one-to-clear */
        SCB_HFSR = SCB_HFSR;
#if MUBL_FAMILY == MUBL_FAMILY_G4
        FLASH_ECCR |= FLASH_ECCR_ECCD;                              /* clear double-bit ECC flag */
#else
        FLASH_CCR(0) = FLASH_CCR_ALL;
#if MUBL_APP_BASE + MUBL_APP_SIZE > MUBL_FLASH_BASE + FLASH_BANK_SIZE
        FLASH_CCR(1) = FLASH_CCR_ALL;
#endif
#endif
        return;
    }
    hal_reset();
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b mubl_fault\n");
}
#else
bool hal_guarded(void (*fn)(void*), void* arg)
{
    fn(arg);
    return true;
}

void HardFault_Handler(void)
{
    hal_reset();
}
#endif

/* ---- System ------------------------------------------------------------------------------- */

uint32_t hal_millis(void)
{
    return ticks;
}

void hal_delay_us(uint32_t us)
{
    const uint32_t reload = SYST_RVR + 1U;
    uint32_t left = us * (stm32_sysclk / 1000000U), last = SYST_CVR;
    while (left != 0U) {
        const uint32_t now = SYST_CVR;
        const uint32_t d = last >= now ? last - now : last + reload - now;
        last = now;
        left = d >= left ? 0U : left - d;
    }
}

void hal_watchdog_refresh(void)
{
    REG32(STM32_IWDG + IWDG_KR) = 0xAAAAU;
}

/* The IWDG runs at reset only with the hardware-watchdog option, and then with its reset period
 * (about 0.5 s), too short for a sector erase: give it MUBL_WATCHDOG_MS.  Otherwise it is left
 * alone unless MUBL_WATCHDOG starts it; programming a stopped IWDG would leave the update pending
 * and could swallow the application's own settings later. */
static void watchdog_init(void)
{
    if (!MUBL_WATCHDOG && !STM32_IWDG_HARDWARE()) return;
    uint32_t reload = MUBL_WATCHDOG_MS / 8U;          /* LSI 32 kHz / 256 */
    if (reload > 0xFFFU) reload = 0xFFFU;
    REG32(STM32_IWDG + IWDG_KR) = 0xCCCCU;            /* start (no effect if running) */
    REG32(STM32_IWDG + IWDG_KR) = 0x5555U;
    REG32(STM32_IWDG + IWDG_PR) = 6U;
    REG32(STM32_IWDG + IWDG_RLR) = reload;
    for (uint32_t i = 0; i < 100000U && REG32(STM32_IWDG + IWDG_SR) != 0U; i++) {
    }
    REG32(STM32_IWDG + IWDG_KR) = 0xAAAAU;
}

void hal_unique_id(uint8_t out[16])
{
    for (uint8_t i = 0; i < 16U; i++) out[i] = i < 12U ? *(const volatile uint8_t*)(uintptr_t)(STM32_UID + i) : 0U;
}

uint32_t hal_idcode(void)
{
    return REG32(STM32_IDCODE);
}

bool hal_hold_pin(void)
{
#ifdef MUBL_HOLD_PIN
    return stm32_pin_read(MUBL_HOLD_PIN) == MUBL_HOLD_LEVEL;
#else
    return false;
#endif
}

void hal_led(bool on)
{
#ifdef MUBL_LED_PIN
    stm32_pin_write(MUBL_LED_PIN, on == MUBL_LED_ON);
#else
    (void)on;
#endif
}

void hal_reset(void)
{
    __asm volatile("dsb" ::: "memory");
    SCB_AIRCR = AIRCR_RESET;
    __asm volatile("dsb" ::: "memory");
    for (;;) {
    }
}

#ifdef MUBL_OUTPUT_PINS
static const struct {
    uint16_t pin;
    uint8_t level;
} outputs[] = {MUBL_OUTPUT_PINS};
#define N_OUTPUTS (sizeof outputs / sizeof outputs[0])
#endif

void hal_init(void)
{
    stm32_clock_init();
#ifdef MUBL_OUTPUT_PINS
    for (uint32_t i = 0; i < N_OUTPUTS; i++) stm32_pin_output(outputs[i].pin, outputs[i].level != 0U);
#endif
    ticks = 0;
    SYST_RVR = stm32_sysclk / 1000U - 1U;
    SYST_CVR = 0;
    SYST_CSR = 7U;                                   /* core clock, interrupt, enable */
    watchdog_init();
#if MUBL_HW_CRC32
    CRC_CLOCK_ENR |= CRC_CLOCK_BIT;
#endif
#ifdef MUBL_HOLD_PIN
    stm32_pin_input(MUBL_HOLD_PIN, MUBL_HOLD_LEVEL ? 2U : 1U);
    for (volatile uint32_t i = 0; i < 1000U; i++) {   /* let the pull settle */
    }
#endif
#ifdef MUBL_LED_PIN
    stm32_pin_output(MUBL_LED_PIN, !MUBL_LED_ON);
#endif
}

/* GPIO ports the board's pins use, to return them to their reset state. */
#define PORT_BIT(p) (1U << MUBL_PIN_PORT(p))
static uint32_t used_ports(void)
{
    uint32_t m = 0;
#if MUBL_DRONECAN
    m |= PORT_BIT(MUBL_CAN_RX_PIN) | PORT_BIT(MUBL_CAN_TX_PIN);
#ifdef MUBL_CAN_STANDBY_PIN
    m |= PORT_BIT(MUBL_CAN_STANDBY_PIN);
#endif
#endif
#ifdef MUBL_UART0_TX_PIN
    m |= PORT_BIT(MUBL_UART0_TX_PIN);
#endif
#ifdef MUBL_UART0_RX_PIN
    m |= PORT_BIT(MUBL_UART0_RX_PIN);
#endif
#ifdef MUBL_UART1_TX_PIN
    m |= PORT_BIT(MUBL_UART1_TX_PIN);
#endif
#ifdef MUBL_UART1_RX_PIN
    m |= PORT_BIT(MUBL_UART1_RX_PIN);
#endif
#ifdef MUBL_LED_PIN
    m |= PORT_BIT(MUBL_LED_PIN);
#endif
#ifdef MUBL_USB_DP_PIN
    m |= PORT_BIT(MUBL_USB_DM_PIN) | PORT_BIT(MUBL_USB_DP_PIN);
#endif
#ifdef MUBL_USB_PULLUP_PIN
    m |= PORT_BIT(MUBL_USB_PULLUP_PIN);
#endif
#ifdef MUBL_HOLD_PIN
    m |= PORT_BIT(MUBL_HOLD_PIN);
#endif
#ifdef MUBL_OUTPUT_PINS
    for (uint32_t i = 0; i < N_OUTPUTS; i++) m |= PORT_BIT(outputs[i].pin);
#endif
    return m;
}

void hal_deinit(void)
{
    __asm volatile("cpsid i" ::: "memory");
    quiesce();
#if MUBL_HW_CRC32
    /* Reset values (the CRC unit has no reset bit on F0/F3), then its clock off. */
    REG32(STM32_CRC + CRC_INIT) = 0xFFFFFFFFU;
    REG32(STM32_CRC + CRC_CR) = CRC_CR_RESET;
    CRC_CLOCK_ENR &= ~CRC_CLOCK_BIT;
#endif
    const uint32_t ports = used_ports();
    for (uint32_t p = 0; p < 11U; p++) {
        if (ports & (1U << p)) GPIO_RESET(p);
    }
    stm32_clock_deinit();
}

void hal_start_app(void)
{
    const uint32_t* app = (const uint32_t*)(uintptr_t)MUBL_APP_BASE;
    const uint32_t sp = app[0], entry = app[1];
#if STM32_CORTEX_M0
    /* No VTOR: copy the application's vectors to the start of SRAM and map SRAM at address 0. */
    uint32_t* ram = (uint32_t*)(uintptr_t)MUBL_RAM_BASE;
    for (uint32_t i = 0; i < MUBL_VECTOR_RESERVE / 4U; i++) ram[i] = app[i];
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_MEM_MODE;
#else
    SCB_VTOR = MUBL_APP_BASE;
#endif
#ifdef STM32_D2_SRAMS
    RCC_AHB2ENR |= STM32_D2_SRAMS;                   /* see family_h7.h */
    (void)RCC_AHB2ENR;
#endif
    /* Interrupts on as after reset: every source is disabled and nothing is pending. */
    __asm volatile(
        "dsb\n"
        "isb\n"
        "msr msp, %0\n"
        "cpsie i\n"
        "bx %1\n" ::"r"(sp), "r"(entry) : "memory");
    for (;;) {
    }
}
