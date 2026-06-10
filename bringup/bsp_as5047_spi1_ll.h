#ifndef BSP_AS5047_SPI1_LL_H
#define BSP_AS5047_SPI1_LL_H

#include <stdint.h>

/**
 * LL SPI1 + DMA1 Ch2(RX)/Ch3(TX) for AS5047 angle read.
 * Cube HAL runs MX_SPI1_Init once; hot path bypasses HAL DMA.
 */
void bsp_as5047_spi1_ll_init(void);
int bsp_as5047_spi1_ll_start_word(const uint16_t *tx, uint16_t *rx);
int bsp_as5047_spi1_ll_restart_word(const uint16_t *tx, uint16_t *rx);
void bsp_as5047_spi1_ll_hw_stop(void);
void bsp_as5047_spi1_ll_dma1_ch2_isr(void);
uint8_t bsp_as5047_spi1_ll_is_busy(void);

#endif
