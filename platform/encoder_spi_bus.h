#ifndef ENCODER_SPI_BUS_H
#define ENCODER_SPI_BUS_H

#include <stdint.h>

struct encoder_spi_bus;

typedef void (*encoder_spi_bus_cb_t)(struct encoder_spi_bus *bus);

typedef struct encoder_spi_bus_ops {
    void (*cs_low)(struct encoder_spi_bus *bus);
    void (*cs_high)(struct encoder_spi_bus *bus);
    void (*hw_init)(struct encoder_spi_bus *bus);
    void (*hw_stop)(struct encoder_spi_bus *bus);
    int (*start_word)(struct encoder_spi_bus *bus, const uint16_t *tx, uint16_t *rx);
    int (*restart_word)(struct encoder_spi_bus *bus, const uint16_t *tx, uint16_t *rx);
    void (*dma_isr)(struct encoder_spi_bus *bus);
} encoder_spi_bus_ops_t;

typedef struct encoder_spi_bus {
    const encoder_spi_bus_ops_t *ops;
    void *spi;
    uint32_t dma_ll_rx_ch;
    uint32_t dma_ll_tx_ch;
    void *cs_port;
    uint16_t cs_pin;
    volatile uint8_t busy;
    void *user_ctx;
    encoder_spi_bus_cb_t on_rx_complete;
    encoder_spi_bus_cb_t on_error;
} encoder_spi_bus_t;

extern const encoder_spi_bus_ops_t encoder_spi_bus_ops_spi1_fast;
extern const encoder_spi_bus_ops_t encoder_spi_bus_ops_generic_g4;

void encoder_spi_bus_cs_low(encoder_spi_bus_t *bus);
void encoder_spi_bus_cs_high(encoder_spi_bus_t *bus);
void encoder_spi_bus_hw_init(encoder_spi_bus_t *bus);
void encoder_spi_bus_hw_stop(encoder_spi_bus_t *bus);
int encoder_spi_bus_start_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx);
int encoder_spi_bus_restart_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx);
uint8_t encoder_spi_bus_is_busy(const encoder_spi_bus_t *bus);
void encoder_spi_bus_dma_isr(encoder_spi_bus_t *bus);

#endif
