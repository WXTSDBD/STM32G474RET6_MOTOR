#include "encoder_spi_bus.h"

#include <stddef.h>

static const encoder_spi_bus_ops_t *encoder_spi_bus_get_ops(const encoder_spi_bus_t *bus)
{
    if (bus == NULL || bus->ops == NULL) {
        return NULL;
    }
    return bus->ops;
}

void encoder_spi_bus_cs_low(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->cs_low != NULL) {
        ops->cs_low(bus);
    }
}

void encoder_spi_bus_cs_high(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->cs_high != NULL) {
        ops->cs_high(bus);
    }
}

void encoder_spi_bus_hw_init(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->hw_init != NULL) {
        ops->hw_init(bus);
    }
}

void encoder_spi_bus_hw_stop(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->hw_stop != NULL) {
        ops->hw_stop(bus);
    }
}

int encoder_spi_bus_start_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops == NULL || ops->start_word == NULL) {
        return -1;
    }
    return ops->start_word(bus, tx, rx);
}

int encoder_spi_bus_restart_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops == NULL || ops->restart_word == NULL) {
        return -1;
    }
    return ops->restart_word(bus, tx, rx);
}

uint8_t encoder_spi_bus_is_busy(const encoder_spi_bus_t *bus)
{
    if (bus == NULL) {
        return 0U;
    }
    return bus->busy;
}

void encoder_spi_bus_dma_isr(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->dma_isr != NULL) {
        ops->dma_isr(bus);
    }
}
