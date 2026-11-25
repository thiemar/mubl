/*
 * A USB host for the simulation: the cable, the host controller and a CDC-ACM driver, talking to
 * src/hal/usb/usb_fs.c through the peripheral model (usb_fs_model.c).
 *
 * While the cable is plugged in and the device has its pull-up on, the host waits out the attach
 * debounce (sim_usb_attach_ms, 100 ms by default), resets the bus and enumerates the device the way
 * Linux does: device descriptor at address 0, SET_ADDRESS, the descriptors, SET_CONFIGURATION,
 * then the CDC line coding and DTR.  From the bus reset on it sends a start-of-frame every
 * millisecond (unless suspended) and moves bytes queued with sim_usb_send to the bulk OUT
 * endpoint, collecting what the device sends on bulk IN for sim_usb_recv.
 *
 * Control transfers run to completion at once, the device's main loop (mubl_step) running
 * whenever the device NAKs, as a host retries.  The device's time does not advance meanwhile.
 * A failed enumeration is retried after 100 ms and counted (sim_usb_failures).
 */
#include <string.h>

#include "hal/host/usb_model.h"
#include "usb_host.h"

#define ADDRESS 5U
#define NAK_LIMIT 64
#define QUEUE 65536

enum { DETACHED = 0, DEBOUNCE = 1, CONFIGURED = 2 };
enum { STALLED = -1, FAILED = -2 };

static struct {
    int plugged, suspended, state;
    uint8_t address;
    uint32_t at, attach_ms;
    int (*step)(void);
    uint32_t enumerations, failures, detaches;
    uint8_t descriptors[3][256];       /* device, configuration, serial number string */
    uint8_t out[QUEUE], in[QUEUE];
    uint32_t out_head, out_n, in_n;
} h;

void usb_host_reset(int (*step)(void))
{
    memset(&h, 0, sizeof h);
    h.attach_ms = 100;
    h.step = step;
}

static usbm_handshake_t in_token(uint8_t ep, uint8_t* buf, uint16_t* len)
{
    usbm_handshake_t r = USBM_NAK;
    for (int i = 0; i < NAK_LIMIT && r == USBM_NAK; i++) {
        r = usbm_in(h.address, ep, buf, len);
        if (r == USBM_NAK && !h.step()) return USBM_TIMEOUT;
    }
    return r;
}

static usbm_handshake_t out_token(uint8_t ep, const uint8_t* buf, uint16_t len)
{
    usbm_handshake_t r = USBM_NAK;
    for (int i = 0; i < NAK_LIMIT && r == USBM_NAK; i++) {
        r = usbm_out(h.address, ep, buf, len);
        if (r == USBM_NAK && !h.step()) return USBM_TIMEOUT;
    }
    return r;
}

/* One control transfer: the data stage length, STALLED or FAILED. */
static int control(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, uint8_t* data)
{
    const uint8_t setup[8] = {type, request, (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)index,
                              (uint8_t)(index >> 8), (uint8_t)length, (uint8_t)(length >> 8)};
    uint8_t packet[64];
    uint16_t done = 0, n;
    usbm_handshake_t r;
    if (usbm_setup(h.address, setup) != USBM_ACK || !h.step()) return FAILED;
    if (length && (type & 0x80U)) {
        for (;;) {
            r = in_token(0, packet, &n);
            if (r == USBM_STALL) return STALLED;
            if (r != USBM_ACK || done + n > length) return FAILED;
            memcpy(data + done, packet, n);
            done = (uint16_t)(done + n);
            if (n < 64U || done == length) break;
        }
        return out_token(0, NULL, 0) == USBM_ACK && h.step() ? done : FAILED;
    }
    while (done < length) {
        n = (uint16_t)(length - done > 64U ? 64U : length - done);
        r = out_token(0, data + done, n);
        if (r == USBM_STALL) return STALLED;
        if (r != USBM_ACK) return FAILED;
        done = (uint16_t)(done + n);
    }
    r = in_token(0, packet, &n);
    if (r == USBM_STALL) return STALLED;
    /* The device sees the status stage complete on its next poll (SET_ADDRESS then applies: hosts
     * leave it 2 ms). */
    return r == USBM_ACK && n == 0U && h.step() ? done : FAILED;
}

static int enumerate(void)
{
    uint8_t* dev = h.descriptors[0];
    uint8_t* cfg = h.descriptors[1];
    uint8_t buf[256];
    static const uint8_t coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};
    h.address = 0;
    usbm_bus_reset();
    if (!h.step()) return 0;
    if (control(0x80, 6, 0x0100, 0, 64, dev) != 18) return 0;
    if (control(0x00, 5, ADDRESS, 0, 0, NULL) != 0) return 0;
    h.address = ADDRESS;
    if (control(0x80, 6, 0x0100, 0, 18, dev) != 18) return 0;
    if (control(0x80, 6, 0x0200, 0, 9, cfg) != 9) return 0;
    const uint16_t total = (uint16_t)(cfg[2] | cfg[3] << 8);
    if (total > 256U || control(0x80, 6, 0x0200, 0, total, cfg) != total) return 0;
    if (control(0x80, 6, 0x0300, 0, 255, buf) < 4) return 0;
    for (uint8_t i = 1; i <= 3U; i++) {
        uint8_t* s = i == 3U ? h.descriptors[2] : buf;
        if (dev[13 + i] != 0U && control(0x80, 6, (uint16_t)(0x0300 | dev[13 + i]), 0x0409, 255, s) < 2) return 0;
    }
    if (control(0x00, 9, 1, 0, 0, NULL) != 0) return 0;
    usbm_reset_host_toggle(0x81);
    usbm_reset_host_toggle(0x02);
    usbm_reset_host_toggle(0x83);
    if (control(0x21, 0x20, 0, 0, 7, (uint8_t*)coding) != 7) return 0;
    if (control(0x21, 0x22, 3, 0, 0, NULL) != 0) return 0;
    return 1;
}

/* Bulk: queued bytes out (until the device NAKs), then everything the device has to send. */
static void bulk(void)
{
    uint8_t packet[64];
    uint16_t n;
    if (h.out_n != 0U) {
        n = (uint16_t)(h.out_n < 64U ? h.out_n : 64U);
        for (uint16_t i = 0; i < n; i++) packet[i] = h.out[(h.out_head + i) % QUEUE];
        const usbm_handshake_t r = usbm_out(h.address, 2, packet, n);
        if (r == USBM_ACK) {
            h.out_head = (h.out_head + n) % QUEUE;
            h.out_n -= n;
        }
    }
    for (int i = 0; i < 16; i++) {
        if (usbm_in(h.address, 1, packet, &n) != USBM_ACK) break;
        for (uint16_t k = 0; k < n && h.in_n < QUEUE; k++) h.in[h.in_n++] = packet[k];
    }
}

void usb_host_frame(uint32_t now)
{
    if (usbm_take_detach() || !h.plugged || !usbm_connected()) {
        if (h.state == CONFIGURED) h.detaches++;
        h.state = DETACHED;
        return;
    }
    if (h.state == DETACHED) {
        h.state = DEBOUNCE;
        h.at = now + h.attach_ms;
        return;
    }
    if (h.state == DEBOUNCE) {
        if ((int32_t)(now - h.at) < 0) return;
        if (enumerate()) {
            h.state = CONFIGURED;
            h.enumerations++;
        } else if (usbm_connected()) {
            h.failures++;
            h.at = now + 100U;
        }
        return;
    }
    if (!h.suspended) usbm_sof();
}

void usb_host_poll(void)
{
    if (h.state == CONFIGURED && usbm_connected()) bulk();
}

/* ---- API for tests --------------------------------------------------------------------- */

void sim_usb_plug(int on) { h.plugged = on; }
void sim_usb_suspend(int on) { h.suspended = on; }
void sim_usb_attach_ms(uint32_t ms) { h.attach_ms = ms; }
int sim_usb_state(void) { return h.state; }
int sim_usb_connected(void) { return usbm_connected(); }
uint32_t sim_usb_enumerations(void) { return h.enumerations; }
uint32_t sim_usb_failures(void) { return h.failures; }
uint32_t sim_usb_detaches(void) { return h.detaches; }
int sim_usb_model_errors(void) { return usbm_take_errors(); }

void sim_usb_descriptor(int which, uint8_t* out)
{
    memcpy(out, h.descriptors[which], sizeof h.descriptors[which]);
}

void sim_usb_send(const uint8_t* d, int n)
{
    for (int i = 0; i < n && h.out_n < QUEUE; i++) {
        h.out[(h.out_head + h.out_n) % QUEUE] = d[i];
        h.out_n++;
    }
}

int sim_usb_recv(uint8_t* out, int max)
{
    const int n = (int)h.in_n < max ? (int)h.in_n : max;
    memcpy(out, h.in, (size_t)n);
    memmove(h.in, h.in + n, h.in_n - (uint32_t)n);
    h.in_n -= (uint32_t)n;
    return n;
}

int sim_usb_pending(void) { return (int)h.out_n; }
