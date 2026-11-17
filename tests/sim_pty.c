/*
 * The simulation in real time behind a pseudo-terminal, so host tools that open serial ports
 * (ArduPilot's uploader.py, tools/mubl_srxl2.py) can talk to it.
 *
 *   sim_pty PORT [APP_IMAGE] [FLASH_DUMP]
 *
 * Prints the terminal's path on stdout, then runs until the simulated bootloader starts the
 * application, a second passes, or it is killed.  On exit it writes the application region to
 * FLASH_DUMP and prints "app <entry>" if the application was started.
 */
#define _XOPEN_SOURCE 600
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

void sim_reset_all(void);
void sim_flash_load(uint32_t offset, const uint8_t* data, uint32_t n);
void sim_flash_get(uint32_t offset, uint8_t* out, uint32_t n);
void sim_power_on(int keep_ram);
int sim_run(uint32_t ms);
int sim_state(void);
uint32_t sim_app_entry(void);
uint32_t sim_app_size(void);
void sim_uart_send(int port, const uint8_t* d, int n, uint32_t baud);
int sim_uart_recv(int port, uint8_t* out, int max, uint32_t baud);

static uint64_t now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000U + (uint64_t)t.tv_nsec / 1000000U;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: sim_pty PORT [APP_IMAGE] [FLASH_DUMP]\n");
        return 2;
    }
    const int port = atoi(argv[1]);
    sim_reset_all();
    if (argc > 2 && strcmp(argv[2], "-") != 0) {
        FILE* f = fopen(argv[2], "rb");
        if (f == NULL) return 1;
        static uint8_t img[4 << 20];
        const size_t n = fread(img, 1, sizeof img, f);
        fclose(f);
        sim_flash_load(0, img, (uint32_t)n);
    }
    const int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0 || grantpt(m) != 0 || unlockpt(m) != 0) {
        perror("sim_pty: no pseudo-terminal");
        printf("\n");
        return 1;
    }
    const char* name = ptsname(m);
    /* Keep the terminal open and raw so nothing echoes before or between clients. */
    const int s = open(name, O_RDWR | O_NOCTTY);
    struct termios t;
    tcgetattr(s, &t);
    t.c_iflag = 0;
    t.c_oflag = 0;
    t.c_lflag = 0;
    t.c_cflag = CS8 | CREAD | CLOCAL;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    tcsetattr(s, TCSANOW, &t);
    fcntl(m, F_SETFL, O_NONBLOCK);
    printf("%s\n", name);
    fflush(stdout);

    sim_power_on(0);
    uint64_t sim_t = now_ms(), app_at = 0;
    for (;;) {
        uint8_t buf[4096];
        const ssize_t n = read(m, buf, sizeof buf);
        if (n > 0) sim_uart_send(port, buf, (int)n, 0);
        const uint64_t real = now_ms();
        while (sim_t < real) {
            sim_t++;
            sim_run(1);
            const int k = sim_uart_recv(port, buf, sizeof buf, 0);
            if (k > 0) (void)!write(m, buf, (size_t)k);
        }
        if (sim_state() == 2 && app_at == 0) app_at = real;
        if (app_at != 0 && real - app_at > 1000U) break;
        if (sim_state() == 0) break;
        usleep(300);
    }
    if (argc > 3) {
        const uint32_t size = sim_app_size();
        uint8_t* flash = malloc(size);
        sim_flash_get(0, flash, size);
        FILE* f = fopen(argv[3], "wb");
        if (f != NULL) {
            fwrite(flash, 1, size, f);
            fclose(f);
        }
        free(flash);
    }
    if (sim_state() == 2) printf("app %08x\n", (unsigned)sim_app_entry());
    return 0;
}
