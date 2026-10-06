/**
 * @file encoder_spi_bus.c
 * @date 2026-10-06
 * @brief 编码器 SPI 总线薄转发。

 *
 * 节拍限制见 encoder_spi_bus.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "encoder_spi_bus.h"

#include <stddef.h>

static const encoder_spi_bus_ops_t *encoder_spi_bus_get_ops(const encoder_spi_bus_t *bus)
{
    if (bus == NULL || bus->ops == NULL) {
        return NULL;
    }
    return bus->ops;
}

/**
 * @brief 拉低片选。
 */
void encoder_spi_bus_cs_low(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->cs_low != NULL) {
        ops->cs_low(bus);
    }
}

/**
 * @brief 拉高片选。
 */
void encoder_spi_bus_cs_high(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->cs_high != NULL) {
        ops->cs_high(bus);
    }
}

/**
 * @brief 配置 SPI/DMA。
 */
void encoder_spi_bus_hw_init(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->hw_init != NULL) {
        ops->hw_init(bus);
    }
}

/**
 * @brief 停 DMA。
 */
void encoder_spi_bus_hw_stop(encoder_spi_bus_t *bus)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops != NULL && ops->hw_stop != NULL) {
        ops->hw_stop(bus);
    }
}

/**
 * @brief 启动一帧。忙则返回非 0。
 */
int encoder_spi_bus_start_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops == NULL || ops->start_word == NULL) {
        return -1;
    }
    return ops->start_word(bus, tx, rx);
}

/**
 * @brief 不停 DMA 再发下一帧。
 */
int encoder_spi_bus_restart_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    const encoder_spi_bus_ops_t *ops = encoder_spi_bus_get_ops(bus);

    if (ops == NULL || ops->restart_word == NULL) {
        return -1;
    }
    return ops->restart_word(bus, tx, rx);
}

/**
 * @brief 1=DMA 进行中。
 */
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
