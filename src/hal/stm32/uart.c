/*
 * USART ports 0 and 1 (MUBL_UARTn_INSTANCE), polled, 8N1, 16x oversampling.  A port without an RX
 * pin is single-wire half-duplex on its TX pin (HDSEL, open drain with pull-up); its receiver is
 * off while it transmits, so it does not hear itself.
 */
#include "hal/hal.h"
#include "stm32.h"

#if MUBL_SERIAL || MUBL_SRXL2

#if STM32_USART_V1
#define CR1 USART1_CR1
#define CR3 USART1_CR3
#define BRR USART1_BRR
#define ISR USART1_SR
#define RDR USART1_DR
#define TDR USART1_DR
#define UE USART1_CR1_UE
#else
#define CR1 USART_CR1
#define CR3 USART_CR3
#define BRR USART_BRR
#define ISR USART_ISR
#define RDR USART_RDR
#define TDR USART_TDR
#define UE USART_CR1_UE
#endif
#define ERRORS (USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)

typedef struct {
    uint32_t base;
    volatile uint32_t* enr;
    volatile uint32_t* rstr;
    uint32_t bit;
    uint16_t tx, rx;             /* pins; rx 0: half duplex */
} port_t;

#define PORT(n, tx, rx)                                                                           \
    {MUBL_CAT3(USART, n, _BASE), &MUBL_CAT3(USART, n, _ENR), &MUBL_CAT3(USART, n, _RSTR),            \
     MUBL_CAT3(USART, n, _BIT), tx, rx}
#ifndef MUBL_UART0_RX_PIN
#define MUBL_UART0_RX_PIN 0
#endif
#ifndef MUBL_UART1_RX_PIN
#define MUBL_UART1_RX_PIN 0
#endif

static const port_t ports[2] = {
#if MUBL_UART0_INSTANCE
    PORT(MUBL_UART0_INSTANCE, MUBL_UART0_TX_PIN, MUBL_UART0_RX_PIN),
#else
    {0, 0, 0, 0, 0, 0},
#endif
#if MUBL_UART1_INSTANCE
    PORT(MUBL_UART1_INSTANCE, MUBL_UART1_TX_PIN, MUBL_UART1_RX_PIN),
#else
    {0, 0, 0, 0, 0, 0},
#endif
};

static uint32_t clock_of(uint8_t port)
{
#if MUBL_UART0_INSTANCE
    if (port == 0U) return MUBL_CAT3(USART, MUBL_UART0_INSTANCE, _CLK);
#endif
#if MUBL_UART1_INSTANCE
    if (port == 1U) return MUBL_CAT3(USART, MUBL_UART1_INSTANCE, _CLK);
#endif
    (void)port;
    return 0;
}

#define R(p, off) REG32(ports[p].base + (off))

void hal_uart_start(uint8_t port, uint32_t baud)
{
    const port_t* p = &ports[port];
    if (p->base == 0U) return;
    *p->enr |= p->bit;
    *p->rstr |= p->bit;
    *p->rstr &= ~p->bit;
    const bool half = p->rx == 0U;
    stm32_pin_af(p->tx, half, half);
    if (!half) stm32_pin_af(p->rx, false, true);
    R(port, BRR) = (clock_of(port) + baud / 2U) / baud;
    R(port, CR3) = half ? USART_CR3_HDSEL : 0U;
    R(port, CR1) = UE | USART_CR1_TE | USART_CR1_RE;
}

int hal_uart_getc(uint8_t port)
{
    if (ports[port].base == 0U) return -1;
    const uint32_t sr = R(port, ISR);
    if (sr & ERRORS) {
#if STM32_USART_V1
        (void)R(port, RDR);                          /* SR then DR clears the flags */
#else
        R(port, USART_ICR) = ERRORS;
        if (sr & USART_ISR_RXNE) (void)R(port, RDR);
#endif
        return -1;                                   /* a damaged byte: drop it */
    }
    if (!(sr & USART_ISR_RXNE)) return -1;
    return (int)(R(port, RDR) & 0xFFU);
}

void hal_uart_write(uint8_t port, const uint8_t* data, uint16_t n)
{
    const port_t* p = &ports[port];
    if (p->base == 0U) return;
    const bool half = p->rx == 0U;
    if (half) R(port, CR1) &= ~USART_CR1_RE;
    while (n--) {
        while (!(R(port, ISR) & USART_ISR_TXE)) {
        }
        R(port, TDR) = *data++;
    }
    while (!(R(port, ISR) & USART_ISR_TC)) {
    }
    if (half) R(port, CR1) |= USART_CR1_RE;
}

void hal_uart_stop(uint8_t port)
{
    const port_t* p = &ports[port];
    if (p->base == 0U) return;
    *p->rstr |= p->bit;
    *p->rstr &= ~p->bit;
    *p->enr &= ~p->bit;
}

#endif
