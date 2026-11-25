/*
 * Spektrum's reference SRXL2 slave (tests/third_party/srxl2) as a shared library, so tests can run
 * it in lockstep with the µBL simulation on the same bus traffic and compare what each sends.
 */
#include <string.h>

#include "spm_srxl.h"

extern SrxlBus srxlBus[SRXL_NUM_OF_BUSES];

static uint8_t tx[8192];
static int tx_n;
static uint32_t baud = 115200;
static uint8_t rx[2 * SRXL_MAX_BUFFER_SIZE];
static int rx_n;

void ref_uart_send(uint8_t uart, uint8_t* buffer, uint8_t length)
{
    (void)uart;
    if (tx_n + length <= (int)sizeof tx) {
        memcpy(&tx[tx_n], buffer, length);
        tx_n += length;
    }
}

void ref_uart_baud(uint8_t uart, uint32_t b)
{
    (void)uart;
    baud = b;
}

int ref_init(uint8_t device_id, uint8_t priority, uint8_t info, uint32_t uid, uint8_t baud_supported)
{
    tx_n = rx_n = 0;
    baud = 115200;
    memset(srxlBus, 0, sizeof srxlBus);
    return srxlInitDevice(device_id, priority, info, uid) && srxlInitBus(0, 1, baud_supported);
}

/* Bytes from the bus; complete packets are parsed as the reference example does. */
void ref_feed(const uint8_t* data, int n)
{
    for (int i = 0; i < n; i++) {
        if (rx_n == 0 && data[i] != SPEKTRUM_SRXL_ID) continue;
        rx[rx_n++] = data[i];
        if (rx_n >= 3 && (rx[2] < 5 || rx[2] > SRXL_MAX_BUFFER_SIZE)) {
            rx_n = 0;
            continue;
        }
        if (rx_n >= 3 && rx_n == rx[2]) {
            srxlParsePacket(0, rx, (uint8_t)rx_n);
            rx_n = 0;
        }
    }
}

void ref_run(int ms) { srxlRun(0, (int16_t)ms); }

int ref_take(uint8_t* out, int max)
{
    const int n = tx_n < max ? tx_n : max;
    memcpy(out, tx, (size_t)n);
    memmove(tx, tx + n, (size_t)(tx_n - n));
    tx_n -= n;
    return n;
}

uint32_t ref_baud(void) { return baud; }
