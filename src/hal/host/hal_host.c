/*
 * Host simulation of the HAL, for tests.  Built as a shared library and driven from Python
 * (ctypes), or linked into tests/sim_pty.c.
 *
 * Flash behaves like the target's: erase sets 0xFF; a granule may be programmed only when it is
 * erased (as on STM32, where reprogramming is an error); writes to the bootloader are refused.
 * With MUBL_FLASH_ECC a granule interrupted while being erased or programmed reads as an
 * uncorrectable ECC error: reading it outside hal_guarded is reported as a fault, as the bus
 * error on hardware would reset the MCU.
 *
 * With sim_blocking_time(1) time passes inside blocking calls as it does on hardware (delays, UART
 * transmission at the line rate, CAN frames, flash erase and program), so code that holds on to a
 * timestamp across them is exercised.
 *
 * Power loss can be injected at the Nth flash operation: the operation is left half done (a
 * random subset of the bytes changed, ECC granules corrupted) and the run stops as if the supply
 * failed.  Misuse is recorded in sim_errors() so tests can assert none happened.
 */
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>

#include "handoff.h"
#include "hal/hal.h"

#define G MUBL_FLASH_GRANULE
#define FLASH_SIZE SIM_FLASH_SIZE

enum { STATE_OFF = 0, STATE_BOOTLOADER = 1, STATE_APP = 2 };
enum { EV_RESET = 1, EV_APP = 2, EV_POWERLOSS = 3, EV_FAULT = 4 };
enum {
    ERR_UNGUARDED_ECC = 1,       /* read an ECC-corrupt granule outside hal_guarded */
    ERR_REPROGRAM = 2,           /* programmed a granule that was not erased */
    ERR_PROTECTED = 4,           /* erased or programmed the bootloader */
    ERR_LISTEN_SEND = 8,         /* sent a CAN frame in listen-only mode */
    ERR_ALIGN = 16,              /* misaligned program */
};

static uint8_t flash[FLASH_SIZE];
static uint8_t corrupt[FLASH_SIZE / G];
static uint8_t ram[MUBL_RAM_SIZE];

static struct {
    int state;
    int errors;
    uint32_t now;
    uint32_t us;                 /* sub-millisecond time passed in blocking calls */
    int blocking;
    uint32_t boots;
    uint32_t ops, inject_at, seed;
    uint32_t app_entry, app_sp;
    uint8_t uid[16];
    int hold;
    int led;
    jmp_buf* top;
    jmp_buf* guard;
    uint32_t px4[5];
    /* CAN */
    int can_on, can_listen, can_err, app_emulation;
    uint8_t can_node;
    uint32_t can_rate, bus_rate;
    dc_frame_t rxq[64], txq[256];
    int rx_head, rx_n, tx_head, tx_n;
    uint32_t app_status_at;
    uint8_t app_tid;
    /* UART */
    struct {
        int on;
        uint32_t baud;
        uint8_t rx[8192], tx[8192];
        int rx_head, rx_n, tx_n;
        uint32_t tx_baud;
    } uart[2];
} sim;

static uint32_t rnd(void)
{
    sim.seed = sim.seed * 1103515245U + 12345U;
    return sim.seed >> 8;
}

/* ---- Memory as seen by the core ---------------------------------------------------------- */

const uint8_t* sim_flash_ptr(uint32_t addr)
{
    return &flash[addr - MUBL_FLASH_BASE];
}

uint8_t* sim_ram_ptr(uint32_t addr)
{
    return &ram[addr - MUBL_RAM_BASE];
}

void sim_flash_read_hook(uint32_t addr, uint32_t n)
{
    if (!MUBL_FLASH_ECC || n == 0U) return;
    const uint32_t off = addr - MUBL_FLASH_BASE;
    for (uint32_t g = off / G; g <= (off + n - 1U) / G; g++) {
        if (!corrupt[g]) continue;
        if (sim.guard != NULL) longjmp(*sim.guard, 1);
        sim.errors |= ERR_UNGUARDED_ECC;
        longjmp(*sim.top, EV_FAULT);
    }
}

/* ---- System -------------------------------------------------------------------------------- */

void hal_init(void)
{
    sim.can_on = 0;
    sim.uart[0].on = sim.uart[1].on = 0;
}

uint32_t hal_millis(void) { return sim.now; }
static void pass_us(uint32_t us)
{
    if (!sim.blocking) return;
    sim.us += us;
    sim.now += sim.us / 1000U;
    sim.us %= 1000U;
}

void hal_delay_us(uint32_t us) { pass_us(us); }
void hal_watchdog_refresh(void) {}
void hal_unique_id(uint8_t out[16]) { memcpy(out, sim.uid, 16); }
uint32_t hal_idcode(void) { return 0x10016413U; }
bool hal_hold_pin(void) { return sim.hold != 0; }
void hal_led(bool on) { sim.led = on; }

void hal_reset(void)
{
    longjmp(*sim.top, EV_RESET);
}

void hal_deinit(void)
{
    sim.can_on = 0;
    sim.uart[0].on = sim.uart[1].on = 0;
}

void hal_start_app(void)
{
    sim.app_sp = mubl_get32(&flash[MUBL_APP_BASE - MUBL_FLASH_BASE]);
    sim.app_entry = mubl_get32(&flash[MUBL_APP_BASE - MUBL_FLASH_BASE + 4]);
    longjmp(*sim.top, EV_APP);
}

bool hal_guarded(void (*fn)(void*), void* arg)
{
    jmp_buf env;
    jmp_buf* outer = sim.guard;
    sim.guard = &env;
    if (setjmp(env) == 0) {
        fn(arg);
        sim.guard = outer;
        return true;
    }
    sim.guard = outer;
    return false;
}

/* ---- Flash ------------------------------------------------------------------------------- */

void hal_flash_sector(uint32_t addr, uint32_t* start, uint32_t* end)
{
    const uint32_t off = addr - MUBL_FLASH_BASE;
#if SIM_GEOMETRY == 1
    /* STM32F4: 4 x 16 KiB, 64 KiB, then 128 KiB sectors */
    uint32_t s, e;
    if (off < 64U * 1024U) {
        s = off & ~(16U * 1024U - 1U);
        e = s + 16U * 1024U;
    } else if (off < 128U * 1024U) {
        s = 64U * 1024U;
        e = 128U * 1024U;
    } else {
        s = off & ~(128U * 1024U - 1U);
        e = s + 128U * 1024U;
    }
    *start = MUBL_FLASH_BASE + s;
    *end = MUBL_FLASH_BASE + e;
#else
    *start = MUBL_FLASH_BASE + (off & ~(SIM_SECTOR - 1U));
    *end = *start + SIM_SECTOR;
#endif
}

static void power_loss_check(void)
{
    sim.ops++;
    if (sim.inject_at != 0U && sim.ops == sim.inject_at) return;   /* caller does the partial op */
}

static int injecting(void)
{
    return sim.inject_at != 0U && sim.ops == sim.inject_at;
}

bool hal_flash_erase(uint32_t start)
{
    uint32_t s, e;
    hal_flash_sector(start, &s, &e);
    if (s != start) return false;
    if (start < MUBL_APP_BASE) {
        sim.errors |= ERR_PROTECTED;
        return false;
    }
    power_loss_check();
    const uint32_t off = s - MUBL_FLASH_BASE, n = e - s;
    pass_us(n / 1024U * 8000U);                /* about 8 ms per KiB (1 s per 128 KiB sector) */
    if (injecting()) {
        for (uint32_t i = 0; i < n; i++) {
            if (rnd() & 1U) flash[off + i] = 0xFF;
        }
        if (MUBL_FLASH_ECC) {
            for (uint32_t g = 0; g < n / G; g++) {
                if ((rnd() & 7U) == 0U) corrupt[off / G + g] = 1;
            }
        }
        longjmp(*sim.top, EV_POWERLOSS);
    }
    memset(&flash[off], 0xFF, n);
    memset(&corrupt[off / G], 0, n / G);
    return true;
}

bool hal_flash_program(uint32_t addr, const uint8_t* data)
{
    const uint32_t off = addr - MUBL_FLASH_BASE;
    if (addr % G != 0U) {
        sim.errors |= ERR_ALIGN;
        return false;
    }
    if (addr < MUBL_APP_BASE) {
        sim.errors |= ERR_PROTECTED;
        return false;
    }
    for (uint32_t i = 0; i < G; i++) {
        if (flash[off + i] != 0xFF || corrupt[off / G]) {
            sim.errors |= ERR_REPROGRAM;
            return false;
        }
    }
    power_loss_check();
    if (injecting()) {
        for (uint32_t i = 0; i < G; i++) {
            if (rnd() & 1U) flash[off + i] &= data[i];
        }
        if (MUBL_FLASH_ECC) corrupt[off / G] = 1;
        longjmp(*sim.top, EV_POWERLOSS);
    }
    for (uint32_t i = 0; i < G; i++) flash[off + i] &= data[i];
    pass_us(G >= 32U ? 100U : 50U);
    return true;
}

#if MUBL_COMPAT_PX4_CAN
void hal_px4_shared_read(uint32_t w[5]) { memcpy(w, sim.px4, sizeof sim.px4); }
void hal_px4_shared_write(const uint32_t w[5])
{
    if (w == NULL) {
        memset(sim.px4, 0, sizeof sim.px4);
    } else {
        memcpy(sim.px4, w, sizeof sim.px4);
    }
}
#endif

/* ---- CAN --------------------------------------------------------------------------------- */

#if MUBL_DRONECAN
bool hal_can_start(uint32_t bitrate, bool listen_only)
{
    sim.can_on = 1;
    sim.can_rate = bitrate;
    sim.can_listen = listen_only;
    sim.can_node = 0;
    sim.rx_n = 0;
    sim.can_err = 0;
    return true;
}

void hal_can_filter(uint8_t node_id) { sim.can_node = node_id; }

bool hal_can_receive(dc_frame_t* f)
{
    if (!sim.can_on || sim.rx_n == 0) return false;
    *f = sim.rxq[sim.rx_head];
    sim.rx_head = (sim.rx_head + 1) % 64;
    sim.rx_n--;
    return true;
}

static void tx_push(const dc_frame_t* f)
{
    if (sim.tx_n == 256) return;
    sim.txq[(sim.tx_head + sim.tx_n) % 256] = *f;
    sim.tx_n++;
}

bool hal_can_send(const dc_frame_t* f)
{
    if (!sim.can_on) return false;
    if (sim.can_listen) {
        sim.errors |= ERR_LISTEN_SEND;
        return false;
    }
    tx_push(f);
    pass_us(130U);                              /* a full frame at 1 Mbit/s */
    return true;
}

bool hal_can_error(void)
{
    const int e = sim.can_err;
    sim.can_err = 0;
    return e != 0;
}

void hal_can_stop(void) { sim.can_on = 0; }
#endif

/* ---- UART -------------------------------------------------------------------------------- */

#if MUBL_SERIAL || MUBL_SRXL2
void hal_uart_start(uint8_t port, uint32_t baud)
{
    sim.uart[port].on = 1;
    sim.uart[port].baud = baud;
    sim.uart[port].rx_n = 0;
}

int hal_uart_getc(uint8_t port)
{
    if (!sim.uart[port].on || sim.uart[port].rx_n == 0) return -1;
    const uint8_t c = sim.uart[port].rx[sim.uart[port].rx_head];
    sim.uart[port].rx_head = (sim.uart[port].rx_head + 1) % 8192;
    sim.uart[port].rx_n--;
    return c;
}

void hal_uart_write(uint8_t port, const uint8_t* data, uint16_t n)
{
    if (!sim.uart[port].on) return;
    if (sim.uart[port].tx_n != 0 && sim.uart[port].tx_baud != sim.uart[port].baud) sim.uart[port].tx_n = 0;
    sim.uart[port].tx_baud = sim.uart[port].baud;
    for (uint16_t i = 0; i < n && sim.uart[port].tx_n < 8192; i++) sim.uart[port].tx[sim.uart[port].tx_n++] = data[i];
    pass_us((uint32_t)((uint64_t)n * 10U * 1000000U / sim.uart[port].baud));
}

void hal_uart_stop(uint8_t port) { sim.uart[port].on = 0; }
#endif

/* ---- Running ----------------------------------------------------------------------------- */

static void boot(void);

/* Run fn under the top-level jump target; handle resets, application starts and power loss. */
static void guarded_run(int initialise)
{
    jmp_buf env;
    jmp_buf* outer = sim.top;
    sim.top = &env;
    sim.guard = NULL;
    const int ev = setjmp(env);
    if (ev == 0) {
        if (initialise) {
            sim.boots++;
            sim.state = STATE_BOOTLOADER;
            mubl_init();
        } else {
            mubl_step();
        }
    } else if (ev == EV_RESET) {
        sim.top = outer;
        boot();
        return;
    } else if (ev == EV_APP) {
        sim.state = STATE_APP;
        sim.app_status_at = sim.now;
    } else {
        sim.state = STATE_OFF;          /* power loss, or a fault that would reset the MCU */
    }
    sim.top = outer;
}

static void boot(void)
{
    guarded_run(1);
}

/* The application, as far as tests need one: a DroneCAN node reporting OPERATIONAL. */
static void app_tick(void)
{
#if MUBL_DRONECAN
    if (!sim.app_emulation || (int32_t)(sim.now - sim.app_status_at) < 0) return;
    sim.app_status_at = sim.now + 1000U;
    const uint8_t* h = &ram[MUBL_HANDOFF_ADDR - MUBL_RAM_BASE];
    if (mubl_get32(h) != MUBL_HANDOFF_TO_APP || h[14] == 0U) return;
    dc_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = (16U << 24) | (341U << 8) | h[14];
    mubl_put32(f.data, sim.now / 1000U);
    f.data[4] = 0;                       /* health OK, mode OPERATIONAL */
    f.data[7] = (uint8_t)(0xC0U | (sim.app_tid++ & 0x1FU));
    f.len = 8;
    tx_push(&f);
#endif
}

/* ---- API for tests --------------------------------------------------------------------- */

void sim_reset_all(void)
{
    memset(&sim, 0, sizeof sim);
    memset(flash, 0xFF, sizeof flash);
    memset(corrupt, 0, sizeof corrupt);
    memset(ram, 0, sizeof ram);
    for (int i = 0; i < 12; i++) sim.uid[i] = (uint8_t)(0xA0 + i);
    sim.seed = 1;
}

void sim_set_uid(const uint8_t* uid) { memcpy(sim.uid, uid, 16); }
void sim_set_hold(int on) { sim.hold = on; }
void sim_app_emulation(int on) { sim.app_emulation = on; }
void sim_blocking_time(int on) { sim.blocking = on; }

void sim_flash_load(uint32_t offset, const uint8_t* data, uint32_t n)
{
    memcpy(&flash[MUBL_APP_BASE - MUBL_FLASH_BASE + offset], data, n);
}

void sim_flash_get(uint32_t offset, uint8_t* out, uint32_t n)
{
    memcpy(out, &flash[MUBL_APP_BASE - MUBL_FLASH_BASE + offset], n);
}

void sim_flash_corrupt(uint32_t offset) { corrupt[(MUBL_APP_BASE - MUBL_FLASH_BASE + offset) / G] = 1; }
int sim_flash_is_corrupt(uint32_t offset) { return corrupt[(MUBL_APP_BASE - MUBL_FLASH_BASE + offset) / G]; }
uint32_t sim_flash_ops(void) { return sim.ops; }

void sim_inject_powerloss(uint32_t op, uint32_t seed)
{
    sim.inject_at = op == 0U ? 0U : sim.ops + op;
    sim.seed = seed | 1U;
}

void sim_ram_set(uint32_t addr, const uint8_t* data, uint32_t n) { memcpy(&ram[addr - MUBL_RAM_BASE], data, n); }
void sim_ram_get(uint32_t addr, uint8_t* out, uint32_t n) { memcpy(out, &ram[addr - MUBL_RAM_BASE], n); }

void sim_power_on(int keep_ram)
{
    if (!keep_ram) {
        for (uint32_t i = 0; i < sizeof ram; i++) ram[i] = (uint8_t)rnd();
    }
    sim.inject_at = 0;
    boot();
}

/* The running application resets the MCU (RAM is kept). */
void sim_app_reset(void)
{
    boot();
}

int sim_run(uint32_t ms)
{
    for (uint32_t i = 0; i < ms && sim.state != STATE_OFF; i++) {
        sim.now++;
        if (sim.state == STATE_APP) {
            app_tick();
            continue;
        }
        for (int k = 0; k < 4 && sim.state == STATE_BOOTLOADER; k++) guarded_run(0);
    }
    return sim.state;
}

int sim_state(void) { return sim.state; }
uint32_t sim_now(void) { return sim.now; }
void sim_set_time(uint32_t ms) { sim.now = ms; }
uint32_t sim_boots(void) { return sim.boots; }
uint32_t sim_app_entry(void) { return sim.app_entry; }
uint32_t sim_app_sp(void) { return sim.app_sp; }
int sim_errors(void) { return sim.errors; }
int sim_led(void) { return sim.led; }
int sim_app_status(void) { return mubl_app()->status; }
int sim_app_format(void) { return mubl_app()->format; }
int sim_stay_reason(void) { return mubl_stay_reason(); }
uint8_t sim_node_id(void) { return mubl_session()->can_node_id; }
uint32_t sim_px4_get(int i) { return sim.px4[i]; }
void sim_px4_set(const uint32_t* w) { memcpy(sim.px4, w, sizeof sim.px4); }

/* Geometry, for tests */
uint32_t sim_app_base(void) { return MUBL_APP_BASE; }
uint32_t sim_app_size(void) { return MUBL_APP_SIZE; }
uint32_t sim_granule(void) { return G; }
uint32_t sim_handoff_addr(void) { return MUBL_HANDOFF_ADDR; }
uint32_t sim_ap_comms_addr(void) { return MUBL_AP_COMMS_ADDR; }
uint32_t sim_board_id(void) { return MUBL_BOARD_ID; }

#if MUBL_SERIAL || MUBL_SRXL2
void sim_uart_send(int port, const uint8_t* d, int n, uint32_t baud)
{
    /* At the wrong rate the bytes would be garbage: drop them.  Rate 0 matches any. */
    if (!sim.uart[port].on || (baud != 0U && sim.uart[port].baud != baud)) return;
    for (int i = 0; i < n && sim.uart[port].rx_n < 8192; i++) {
        sim.uart[port].rx[(sim.uart[port].rx_head + sim.uart[port].rx_n) % 8192] = d[i];
        sim.uart[port].rx_n++;
    }
}

int sim_uart_recv(int port, uint8_t* out, int max, uint32_t baud)
{
    if (baud != 0U && sim.uart[port].tx_baud != baud) {
        sim.uart[port].tx_n = 0;
        return 0;
    }
    int n = sim.uart[port].tx_n < max ? sim.uart[port].tx_n : max;
    memcpy(out, sim.uart[port].tx, (size_t)n);
    memmove(sim.uart[port].tx, sim.uart[port].tx + n, (size_t)(sim.uart[port].tx_n - n));
    sim.uart[port].tx_n -= n;
    return n;
}

uint32_t sim_uart_baud(int port) { return sim.uart[port].on ? sim.uart[port].baud : 0U; }
#endif

#if MUBL_DRONECAN
void sim_can_bus_bitrate(uint32_t rate) { sim.bus_rate = rate; }
uint32_t sim_can_bitrate(void) { return sim.can_on ? sim.can_rate : 0U; }
int sim_can_listen_only(void) { return sim.can_listen; }

/* sfwa_dronecan.vbus.CNodeLibrary interface */
int sim_rx(uint32_t id, const uint8_t* data, uint8_t len, uint8_t fd, uint64_t now_us)
{
    (void)fd;
    (void)now_us;
    if (sim.state != STATE_BOOTLOADER || !sim.can_on || len == 0U || len > DC_FRAME_MAX) return 0;
    if (sim.bus_rate != 0U && sim.bus_rate != sim.can_rate) {
        sim.can_err = 1;                 /* wrong bitrate: the controller sees errors, no frames */
        return 0;
    }
    if (sim.can_node != 0U && (!(id & 0x80U) || ((id >> 8) & 0x7FU) != sim.can_node)) return 0;
    if (sim.rx_n == 64) return 0;
    dc_frame_t* f = &sim.rxq[(sim.rx_head + sim.rx_n) % 64];
    memset(f, 0, sizeof *f);
    f->id = id;
    f->len = len;
    memcpy(f->data, data, len);
    sim.rx_n++;
    return 1;
}

int sim_tx(uint32_t* id, uint8_t* data, uint8_t* fd)
{
    if (sim.tx_n == 0) return 0;
    const dc_frame_t* f = &sim.txq[sim.tx_head];
    sim.tx_head = (sim.tx_head + 1) % 256;
    sim.tx_n--;
    *id = f->id;
    *fd = (f->flags & DC_FRAME_FLAG_FD) ? 1 : 0;
    memcpy(data, f->data, f->len);
    return f->len;
}

void sim_tick(uint64_t now_us)
{
    const uint32_t target = (uint32_t)(now_us / 1000U);
    if (sim.state == STATE_OFF) {
        sim.now = target;
        return;
    }
    if ((int32_t)(target - sim.now) > 0) sim_run(target - sim.now);
}
#endif
