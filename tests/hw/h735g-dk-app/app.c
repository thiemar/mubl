/*
 * Hardware test application for the h735g-dk bootloader (STM32H735G-DK).
 *
 * Runs from 0x08020000 on the reset clocks (HSI, 64 MHz) and reports on the ST-LINK's virtual COM
 * port (USART3, PD8/PD9, 115200 8N1): what the bootloader handed over, VTOR, and a tick every
 * second.  LED1 (PC3) and LED2 (PC2) alternate.  Keys on the console:
 *   i  print the report again
 *   r  reset (the bootloader decides what happens next)
 *   b  ask the bootloader to stay (handoff request, link USB), then reset
 */
#include <stdbool.h>
#include <stdint.h>

#define MUBL_HANDOFF_ADDR 0x20000000U
#include "mubl_app.h"

#ifndef APP_VERSION
#define APP_VERSION 1
#endif
#ifndef GIT_HASH
#define GIT_HASH 0
#endif

#define REG(a) (*(volatile uint32_t*)(uintptr_t)(a))

#define RCC 0x58024400U
#define RCC_AHB4ENR REG(RCC + 0x0E0U)
#define RCC_APB1LENR REG(RCC + 0x0E8U)
#define RCC_RSR REG(RCC + 0x0D0U)
#define GPIOC 0x58020800U
#define GPIOD 0x58020C00U
#define GPIO_MODER 0x00U
#define GPIO_AFRH 0x24U
#define GPIO_BSRR 0x18U
#define USART3 0x40004800U
#define USART_CR1 REG(USART3 + 0x00U)
#define USART_BRR REG(USART3 + 0x0CU)
#define USART_ISR REG(USART3 + 0x1CU)
#define USART_RDR REG(USART3 + 0x24U)
#define USART_TDR REG(USART3 + 0x28U)
#define SYST_CSR REG(0xE000E010U)
#define SYST_RVR REG(0xE000E014U)
#define SYST_CVR REG(0xE000E018U)
#define SCB_VTOR REG(0xE000ED08U)
#define SCB_AIRCR REG(0xE000ED0CU)
#define PWR_CR3 REG(0x58024800U + 0x0CU)

struct app_descriptor {
    uint8_t sig[8];
    uint32_t image_crc1, image_crc2, image_size, git_hash;
    uint8_t version_major, version_minor;
    uint16_t board_id;
    uint8_t reserved[8];
};
__attribute__((section(".app_descriptor"), used)) const struct app_descriptor app_descriptor = {
    {0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06}, 0, 0, 0, GIT_HASH, APP_VERSION, 0, 7352, {0}};

static void put(char c)
{
    while (!(USART_ISR & (1U << 7))) {
    }
    USART_TDR = (uint8_t)c;
}

static void puts_(const char* s)
{
    while (*s) {
        if (*s == '\n') put('\r');
        put(*s++);
    }
}

static void hex(uint32_t v)
{
    puts_("0x");
    for (int i = 28; i >= 0; i -= 4) put("0123456789ABCDEF"[(v >> i) & 0xFU]);
}

static void dec(uint32_t v)
{
    char b[11];
    int n = 0;
    do {
        b[n++] = (char)('0' + v % 10U);
        v /= 10U;
    } while (v);
    while (n) put(b[--n]);
}

static mubl_info_t info;
static bool have_info;
static uint32_t reset_flags;

static void report(void)
{
    puts_("\nmubl-test-app v");
    dec(APP_VERSION);
    puts_(" running: VTOR ");
    hex(SCB_VTOR);
    puts_(" PWR_CR3 ");
    hex(PWR_CR3);
    puts_(" RCC_RSR ");
    hex(reset_flags);
    puts_("\nhandoff: ");
    if (have_info) {
        puts_("link ");
        dec(info.link);
        puts_(info.link == MUBL_LINK_USB ? " (USB)" : info.link == 0 ? " (none)" : "");
        puts_(" boot_attempts ");
        dec(info.boot_attempts);
    } else {
        puts_("none");
    }
    puts_("\n");
}

static void reset(void)
{
    puts_("resetting\n");
    while (!(USART_ISR & (1U << 6))) {               /* TC: last character out */
    }
    __asm volatile("dsb" ::: "memory");
    SCB_AIRCR = (0x5FAU << 16) | (1U << 2);
    for (;;) {
    }
}

int main(void)
{
    have_info = mubl_app_info(&info);
    mubl_app_confirm();
    reset_flags = RCC_RSR;
    RCC_RSR = 1U << 16;                              /* RMVF: clear the reset flags */

    RCC_AHB4ENR |= (1U << 2) | (1U << 3);            /* GPIOC, GPIOD */
    RCC_APB1LENR |= 1U << 18;                        /* USART3 */
    (void)RCC_APB1LENR;
    /* LEDs PC2, PC3: outputs, both off (high) */
    REG(GPIOC + GPIO_BSRR) = (1U << 2) | (1U << 3);
    REG(GPIOC + GPIO_MODER) = (REG(GPIOC + GPIO_MODER) & ~(0xFU << 4)) | (0x5U << 4);
    /* PD8 TX, PD9 RX: AF7 */
    REG(GPIOD + GPIO_AFRH) = (REG(GPIOD + GPIO_AFRH) & ~0xFFU) | 0x77U;
    REG(GPIOD + GPIO_MODER) = (REG(GPIOD + GPIO_MODER) & ~(0xFU << 16)) | (0xAU << 16);
    USART_BRR = (64000000U + 57600U) / 115200U;      /* PCLK1 = HSI 64 MHz after reset */
    USART_CR1 = (1U << 3) | (1U << 2) | 1U;          /* TE, RE, UE */

    SYST_RVR = 64000U - 1U;                          /* 1 ms */
    SYST_CVR = 0;
    SYST_CSR = 5U;                                   /* core clock, no interrupt */

    report();
    uint32_t ms = 0, ticks = 0;
    for (;;) {
        if (SYST_CSR & (1U << 16)) {
            ms++;
            if (ms % 500U == 0U) {
                const bool a = (ms / 500U) & 1U;
                REG(GPIOC + GPIO_BSRR) = a ? (1U << 3) | (1U << 18) : (1U << 2) | (1U << 19);
            }
            if (ms % 1000U == 0U) {
                puts_("tick ");
                dec(++ticks);
                puts_("\n");
            }
        }
        if (USART_ISR & (1U << 5)) {
            const char c = (char)USART_RDR;
            if (c == 'i') report();
            if (c == 'r') reset();
            if (c == 'b') {
                puts_("asking the bootloader to stay\n");
                mubl_app_request(MUBL_LINK_USB, 0, 0, 0, 0, 0, 0);
                reset();
            }
        }
        if (USART_ISR & (1U << 3)) REG(USART3 + 0x20U) = 1U << 3;   /* ORE: clear (ICR) */
    }
}

/* ---- start-up ---- */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

__attribute__((noreturn)) void reset_handler(void)
{
    for (uint32_t *s = &_sidata, *d = &_sdata; d < &_edata;) *d++ = *s++;
    for (uint32_t* d = &_sbss; d < &_ebss;) *d++ = 0;
    main();
    for (;;) {
    }
}

static void fault(void)
{
    for (;;) {
    }
}

__attribute__((section(".isr_vector"), used)) void (*const vectors[16])(void) = {
    (void (*)(void))(uintptr_t)&_estack, reset_handler, fault, fault, fault, fault, fault, 0, 0, 0, 0,
    fault, fault, 0, fault, fault};
