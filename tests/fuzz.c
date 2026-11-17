/*
 * Random traffic on every link of the simulation, for AddressSanitizer and UBSan.
 *
 *   fuzz [iterations] [seed]
 *
 * Each iteration powers the simulation up with random RAM (sometimes a well-formed handoff) and
 * random or partly valid flash, then sends a mix of random bytes and well-formed-but-random serial
 * commands, SRXL2 packets (valid CRC, random DFU operations) and DroneCAN transfers (valid framing
 * and transfer CRC, random payload) while time passes.  It fails if the simulation records misuse
 * (an unguarded ECC read, reprogramming, a write to the bootloader, sending while listening).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sim_reset_all(void);
void sim_flash_load(uint32_t offset, const uint8_t* data, uint32_t n);
void sim_flash_corrupt(uint32_t offset);
void sim_ram_set(uint32_t addr, const uint8_t* data, uint32_t n);
void sim_power_on(int keep_ram);
void sim_app_reset(void);
int sim_run(uint32_t ms);
int sim_state(void);
int sim_errors(void);
uint32_t sim_handoff_addr(void);
uint32_t sim_app_base(void);
uint32_t sim_app_size(void);
void sim_inject_powerloss(uint32_t op, uint32_t seed);
void sim_uart_send(int port, const uint8_t* d, int n, uint32_t baud);
int sim_uart_recv(int port, uint8_t* out, int max, uint32_t baud);
int sim_rx(uint32_t id, const uint8_t* data, uint8_t len, uint8_t fd, uint64_t now_us);
int sim_tx(uint32_t* id, uint8_t* data, uint8_t* fd);
void sim_can_bus_bitrate(uint32_t rate);
void sim_blocking_time(int on);
uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n);
uint16_t crc16_xmodem(uint16_t crc, const uint8_t* p, uint32_t n);

static uint32_t state = 1;
static uint32_t rnd(void)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}
static uint32_t below(uint32_t n) { return n ? rnd() % n : 0; }

static void random_bytes(uint8_t* p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) p[i] = (uint8_t)rnd();
}

/* CRC-16-CCITT-FALSE with a DroneCAN type seed: any seed will do for fuzzing transfers whose CRC
 * the transport checks; use the real seeds of the accepted types so some get through. */
static uint16_t crc16_ccitt(uint16_t crc, const uint8_t* p, uint32_t n)
{
    while (n--) {
        crc ^= (uint16_t)(*p++ << 8);
        for (int i = 0; i < 8; i++) crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

static void can_transfer(void)
{
    static const struct { uint16_t id, seed; uint8_t service, request; } types[] = {
        {1, 0xD9A7, 1, 1}, {5, 0x1F7F, 1, 1}, {40, 0x729E, 1, 1}, {48, 0x2F12, 1, 0}, {1, 0xF258, 0, 0},
    };
    const int t = (int)below(5);
    uint8_t payload[300];
    const uint32_t len = below(below(4) == 0 ? 300 : 20);
    random_bytes(payload, len);
    const uint8_t src = (uint8_t)(1 + below(127)), dst = (uint8_t)below(128), tid = (uint8_t)below(32);
    uint32_t id = (uint32_t)below(32) << 24 | src;
    if (types[t].service) {
        id |= (uint32_t)types[t].id << 16 | (types[t].request ? 0x8000U : 0U) | (uint32_t)dst << 8 | 0x80U;
    } else {
        id |= (uint32_t)types[t].id << 8;
    }
    uint8_t frame[8];
    if (len <= 7) {
        memcpy(frame, payload, len);
        frame[len] = (uint8_t)(0xC0 | tid);
        sim_rx(id, frame, (uint8_t)(len + 1), 0, 0);
        return;
    }
    uint8_t stream[302];
    const uint16_t crc = crc16_ccitt(types[t].seed, payload, len);
    stream[0] = (uint8_t)crc;
    stream[1] = (uint8_t)(crc >> 8);
    memcpy(stream + 2, payload, len);
    const uint32_t total = len + 2;
    uint8_t toggle = 0;
    for (uint32_t pos = 0; pos < total;) {
        const uint32_t n = total - pos > 7 ? 7 : total - pos;
        memcpy(frame, stream + pos, n);
        frame[n] = (uint8_t)((pos == 0 ? 0x80 : 0) | (pos + n >= total ? 0x40 : 0) | toggle << 5 | tid);
        if (below(50) != 0) sim_rx(id, frame, (uint8_t)(n + 1), 0, 0);     /* sometimes a frame is lost */
        pos += n;
        toggle ^= 1;
        if (below(8) == 0) sim_run(1);
    }
}

static void serial_command(int port)
{
    static const uint8_t ops[] = {0x21, 0x22, 0x23, 0x27, 0x29, 0x2A, 0x2B, 0x2C, 0x2E, 0x2F, 0x30, 0x33, 0x40};
    uint8_t b[260];
    uint32_t n = 0;
    b[n++] = ops[below(sizeof ops)];
    if (b[0] == 0x22) b[n++] = (uint8_t)below(8);
    if (b[0] == 0x27) {
        const uint8_t k = (uint8_t)(below(64) * 4);
        b[n++] = k;
        random_bytes(b + n, k);
        n += k;
    }
    if (b[0] == 0x2A || b[0] == 0x2B || b[0] == 0x33) {
        random_bytes(b + n, 4);
        if (b[0] == 0x33) b[n] = 0, b[n + 1] = 0xC2, b[n + 2] = 0x01, b[n + 3] = 0;   /* 115200 */
        n += 4;
    }
    b[n++] = below(20) ? 0x20 : (uint8_t)rnd();
    sim_uart_send(port, b, (int)n, 0);
}

static void srxl2_packet(int port)
{
    uint8_t p[80];
    uint32_t n;
    if (below(3) == 0) {                               /* handshake */
        n = 14;
        p[1] = 0x21;
        random_bytes(p + 3, 9);
        if (below(2)) p[4] = below(2) ? 0x40 : 0xFF;
    } else {                                           /* update packet */
        const uint8_t op = (uint8_t)(1 + below(6));
        uint32_t data = op == 3 ? 4 + below(65) : op == 4 ? 8 : op == 2 ? 4 : below(4);
        n = 9 + data;
        p[1] = 0xB0;
        p[3] = below(4) ? 0x40 : (uint8_t)rnd();
        p[4] = 0x21;
        p[5] = op;
        p[6] = (uint8_t)below(4);
        random_bytes(p + 7, data);
        if (op == 3 && below(2)) p[7] = p[8] = p[9] = p[10] = 0;   /* offset 0 */
    }
    p[0] = 0xA6;
    p[2] = (uint8_t)n;
    const uint16_t crc = crc16_xmodem(0, p, n - 2);
    p[n - 2] = (uint8_t)(crc >> 8);
    p[n - 1] = (uint8_t)crc;
    if (below(30) == 0) p[below(n)] ^= 1;
    sim_uart_send(port, p, (int)n, 0);
}

int main(int argc, char** argv)
{
    const int iterations = argc > 1 ? atoi(argv[1]) : 300;
    state = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 0) | 1U : 0x12345U;
    static uint8_t img[64 * 1024];
    uint8_t buf[8192];
    for (int it = 0; it < iterations; it++) {
        sim_reset_all();
        const uint32_t room = sim_app_size() < sizeof img ? sim_app_size() : (uint32_t)sizeof img;
        const uint32_t size = 512 + below(room - 512);
        random_bytes(img, size);
        if (below(2)) {                                /* plausible vectors */
            const uint32_t sp = 0x20004000U, entry = sim_app_base() + 0x401U;
            memcpy(img, &sp, 4);
            memcpy(img + 4, &entry, 4);
        }
        if (below(3) == 0) memcpy(img + 0x200, "\x40\xa2\xe4\xf1\x64\x68\x91\x06", 8);
        if (below(3) == 0) memcpy(img + 0x300, "APDesc00", 8);
        sim_flash_load(0, img, size);
        if (below(10) == 0) sim_flash_corrupt(below(4) ? below(size) : 32);   /* 32: an interrupted first write */
        uint8_t ram[256];
        random_bytes(ram, sizeof ram);
        if (below(3) == 0) {                           /* a well-formed handoff with random contents */
            const uint16_t len = (uint16_t)(20 + below(201));
            ram[0] = 0x6D, ram[1] = 0x75, ram[2] = 0x42, ram[3] = 0x4C;
            ram[8] = (uint8_t)len, ram[9] = (uint8_t)(len >> 8), ram[10] = 1;
            ram[27] = (uint8_t)rnd();
            const uint32_t crc = ~crc32_update(~0U, ram + 8, len);
            memcpy(ram + 4, &crc, 4);
        }
        sim_ram_set(sim_handoff_addr(), ram, sizeof ram);
        sim_can_bus_bitrate(below(2) ? 0 : 500000);
        sim_blocking_time((int)below(2));
        sim_power_on(1);
        if (below(20) == 0) sim_inject_powerloss(1 + below(200), rnd());
        for (int step = 0; step < 400 && sim_state() != 0; step++) {
            switch (below(10)) {
            case 0: random_bytes(buf, 1 + below(64)); sim_uart_send((int)below(2), buf, 1 + (int)below(64), 0); break;
            case 1: case 2: serial_command((int)below(2)); break;
            case 3: case 4: srxl2_packet((int)below(2)); break;
            case 5: case 6: can_transfer(); break;
            case 7: {
                uint8_t f[8];
                random_bytes(f, 8);
                sim_rx(rnd() & 0x1FFFFFFFU, f, (uint8_t)(1 + below(8)), 0, 0);
                break;
            }
            default: sim_run(1 + below(50)); break;
            }
            sim_run(1);
            if (sim_state() == 2 && below(4) == 0) sim_app_reset();
            while (sim_uart_recv(0, buf, sizeof buf, 0) > 0) {}
            while (sim_uart_recv(1, buf, sizeof buf, 0) > 0) {}
            uint32_t id;
            uint8_t fd;
            while (sim_tx(&id, buf, &fd) > 0) {}
        }
        if (sim_errors() != 0) {
            fprintf(stderr, "iteration %d: simulation errors 0x%x\n", it, sim_errors());
            return 1;
        }
    }
    printf("%d iterations clean\n", iterations);
    return 0;
}
