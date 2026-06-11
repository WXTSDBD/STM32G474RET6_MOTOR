#ifndef BOARD_ENCODER_H
#define BOARD_ENCODER_H

#include "encoder.h"
#include "encoder_spi_bus.h"
#include "as5047.h"

extern encoder_t enc_m1;
extern encoder_spi_bus_t enc_m1_bus;
extern as5047_ctx_t enc_m1_as5047;

void board_encoder_m1_init(void);
void board_encoder_m1_dma_isr(void);

/* M2 placeholder: SPI3 + blocking HAL until DMA bus is wired */
extern encoder_t enc_m2;

#endif
