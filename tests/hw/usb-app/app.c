/*
 * Hardware test application with a USB console, for µBL boards with the OTG core (H7, F4).
 *
 * It starts from the state the bootloader leaves (reset clocks, HSI 64 MHz on H7), brings up USB
 * with µBL's own driver as a CDC-ACM device with the product string "mubl test app", and answers
 * keys on that port:
 *   i  report: version, board, VTOR, what the bootloader handed over, uptime
 *   r  detach and reset (the bootloader decides what happens next)
 *   b  ask the bootloader to stay (handoff request, link USB), detach and reset
 * The board's LED (MUBL_LED_PIN) blinks at 1 Hz.  Its first report is sent when the host first
 * opens the port and sends a key, so nothing is lost before enumeration.
 */
#include "hal/hal.h"
#include "stm32.h"
#include "mubl_app.h"

#ifndef APP_VERSION
#define APP_VERSION 1
#endif
#ifndef GIT_HASH
#define GIT_HASH 0
#endif

struct app_descriptor {
    uint8_t sig[8];
    uint32_t image_crc1, image_crc2, image_size, git_hash;
    uint8_t version_major, version_minor;
    uint16_t board_id;
    uint8_t reserved[8];
};
__attribute__((section(".app_descriptor"), used)) const struct app_descriptor app_descriptor = {
    {0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06}, 0, 0, 0, GIT_HASH, APP_VERSION, 0, MUBL_BOARD_ID, {0}};

#define SYST_CSR_ REG32(0xE000E010U)
#define SYST_RVR_ REG32(0xE000E014U)
#define SYST_CVR_ REG32(0xE000E018U)
#define SCB_VTOR_ REG32(0xE000ED08U)
#define SCB_AIRCR_ REG32(0xE000ED0CU)
#define CORE_HZ 64000000U

static volatile uint32_t ms;
static mubl_info_t info;
static bool have_info;

void systick_handler(void) { ms++; }

/* What µBL's USB driver needs from the HAL. */
void hal_delay_us(uint32_t us)
{
    const uint32_t reload = SYST_RVR_ + 1U;
    uint32_t left = us * (CORE_HZ / 1000000U), last = SYST_CVR_;
    while (left != 0U) {
        const uint32_t now = SYST_CVR_;
        const uint32_t d = last >= now ? last - now : last + reload - now;
        last = now;
        left = d >= left ? 0U : left - d;
    }
}

void hal_unique_id(uint8_t out[16])
{
    for (uint8_t i = 0; i < 16U; i++) out[i] = i < 12U ? *(const volatile uint8_t*)(uintptr_t)(STM32_UID + i) : 0U;
}

/* ---- output ---- */
static char line[256];
static uint16_t len;

static void add(const char* s)
{
    while (*s && len < sizeof line) line[len++] = *s++;
}

static void add_dec(uint32_t v)
{
    char b[11];
    int n = 0;
    do {
        b[n++] = (char)('0' + v % 10U);
        v /= 10U;
    } while (v);
    while (n && len < sizeof line) line[len++] = b[--n];
}

static void add_hex(uint32_t v)
{
    add("0x");
    for (int i = 28; i >= 0 && len < sizeof line; i -= 4) line[len++] = "0123456789ABCDEF"[(v >> i) & 0xFU];
}

static void flush_line(void)
{
    const uint32_t start = ms;
    uint16_t done = 0;
    while (done < len && ms - start < 500U) {
        const uint16_t left = (uint16_t)(len - done), n = left > 64U ? 64U : left;
        if (hal_usb_write((const uint8_t*)line + done, n)) done = (uint16_t)(done + n);
        (void)hal_usb_poll();
    }
    len = 0;
}

static void report(void)
{
    add("mubl test app v");
    add_dec(APP_VERSION);
    add(" on " MUBL_BOARD_NAME " (board ID ");
    add_dec(MUBL_BOARD_ID);
    add(")\r\nVTOR ");
    add_hex(SCB_VTOR_);
    add(", handoff: ");
    if (have_info) {
        add("link ");
        add_dec(info.link);
        add(info.link == MUBL_LINK_USB ? " (USB)" : info.link == MUBL_LINK_DRONECAN ? " (DroneCAN)"
            : info.link == MUBL_LINK_SERIAL ? " (serial)" : info.link == 0U ? " (none)" : "");
        add(", boot_attempts ");
        add_dec(info.boot_attempts);
    } else {
        add("none");
    }
    add("\r\nuptime ");
    add_dec(ms);
    add(" ms\r\n");
    flush_line();
}

static void reset(void)
{
    add("resetting\r\n");
    flush_line();
    hal_usb_stop();                                  /* flush, detach, peripheral off */
    __asm volatile("dsb" ::: "memory");
    SCB_AIRCR_ = (0x5FAU << 16) | (1U << 2);
    for (;;) {
    }
}

int main(void)
{
    have_info = mubl_app_info(&info);
    mubl_app_confirm();
    SYST_RVR_ = CORE_HZ / 1000U - 1U;
    SYST_CVR_ = 0;
    SYST_CSR_ = 7U;                                  /* core clock, interrupt, enable */
#ifdef MUBL_LED_PIN
    stm32_pin_output(MUBL_LED_PIN, !MUBL_LED_ON);
#endif
    hal_usb_start();
    uint32_t blink = 0;
    for (;;) {
        (void)hal_usb_poll();
        const int c = hal_usb_getc();
        if (c == 'i') report();
        if (c == 'r') reset();
        if (c == 'b') {
            add("asking the bootloader to stay\r\n");
            mubl_app_request(MUBL_LINK_USB, 0, 0, 0, 0, 0, 0);
            reset();
        }
#ifdef MUBL_LED_PIN
        if (ms - blink >= 500U) {
            blink = ms;
            stm32_pin_write(MUBL_LED_PIN, ((blink / 500U) & 1U) ? MUBL_LED_ON : !MUBL_LED_ON);
        }
#else
        (void)blink;
#endif
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
    fault, fault, 0, fault, systick_handler};
