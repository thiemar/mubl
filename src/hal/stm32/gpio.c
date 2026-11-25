/*
 * Pins.  A pin is MUBL_PIN(port letter, number, alternate function).
 */
#include "stm32.h"

static void field(uint32_t base, uint32_t reg, uint8_t n, uint8_t width, uint32_t value)
{
    const uint32_t shift = (uint32_t)n * width, mask = ((1U << width) - 1U) << shift;
    REG32(base + reg) = (REG32(base + reg) & ~mask) | (value << shift);
}

static uint32_t port_of(uint16_t pin)
{
    const uint32_t port = MUBL_PIN_PORT(pin);
    GPIO_ENABLE(port);
    (void)REG32(GPIO_BASE(port) + GPIO_MODER);     /* let the clock settle before configuring */
    return GPIO_BASE(port);
}

void stm32_pin_af(uint16_t pin, bool open_drain, bool pull_up)
{
    const uint32_t g = port_of(pin);
    const uint8_t n = MUBL_PIN_NUM(pin);
    field(g, GPIO_AFRL + (n >= 8U ? 4U : 0U), n & 7U, 4, MUBL_PIN_AF(pin));
    field(g, GPIO_OTYPER, n, 1, open_drain);
    field(g, GPIO_OSPEEDR, n, 2, STM32_GPIO_HIGH_SPEED);
    field(g, GPIO_PUPDR, n, 2, pull_up ? 1U : 0U);
    field(g, GPIO_MODER, n, 2, 2U);                /* alternate function */
}

void stm32_pin_output(uint16_t pin, bool level)
{
    const uint32_t g = port_of(pin);
    const uint8_t n = MUBL_PIN_NUM(pin);
    stm32_pin_write(pin, level);
    field(g, GPIO_OTYPER, n, 1, 0);
    field(g, GPIO_MODER, n, 2, 1U);
}

void stm32_pin_input(uint16_t pin, uint8_t pull)
{
    const uint32_t g = port_of(pin);
    const uint8_t n = MUBL_PIN_NUM(pin);
    field(g, GPIO_PUPDR, n, 2, pull);
    field(g, GPIO_MODER, n, 2, 0U);
}

bool stm32_pin_read(uint16_t pin)
{
    return (REG32(GPIO_BASE(MUBL_PIN_PORT(pin)) + GPIO_IDR) >> MUBL_PIN_NUM(pin)) & 1U;
}

void stm32_pin_write(uint16_t pin, bool level)
{
    REG32(GPIO_BASE(MUBL_PIN_PORT(pin)) + GPIO_BSRR) = 1U << (MUBL_PIN_NUM(pin) + (level ? 0U : 16U));
}

void stm32_pin_reset(uint16_t pin)
{
    GPIO_RESET(MUBL_PIN_PORT(pin));
}
