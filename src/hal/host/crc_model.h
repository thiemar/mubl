/* Host model of the STM32 CRC unit: see crc_model.c. */
#ifndef MUBL_CRC_MODEL_H
#define MUBL_CRC_MODEL_H

#include <stdbool.h>
#include <stdint.h>

void crc_model_clock(bool on);
extern unsigned crc_model_errors;

/* The register accesses src/hal/stm32/crc_hw.c makes with MUBL_CRC_MODEL. */
void model_start(uint32_t init, uint32_t cr);
void model_put8(uint8_t b);
void model_put32(uint32_t w);
uint32_t model_get(void);

#endif
