/**
 * SPI1 + DMA1 Ch2/Ch3 fast path (M1 @ 20kHz).
 * Hard-coded LL flags — no per-transfer channel switch.
 */
#include "encoder_spi_bus.h"

#include <stddef.h>

#include "hal_bridge.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"

static SPI_TypeDef *fast_spi(const encoder_spi_bus_t *bus)
{
    return (SPI_TypeDef *)bus->spi;
}

static void fast_cs_low(encoder_spi_bus_t *bus)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)bus->cs_port;

    port->BSRR = ((uint32_t)bus->cs_pin << 16U);
}

static void fast_cs_high(encoder_spi_bus_t *bus)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)bus->cs_port;

    port->BSRR = (uint32_t)bus->cs_pin;
}

static void fast_ll_dma_clear_tc2(void)
{
    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);
    }
    if (LL_DMA_IsActiveFlag_GI2(DMA1)) {
        LL_DMA_ClearFlag_GI2(DMA1);
    }
}

static void fast_ll_dma_clear_ch2_flags_full(void)
{
    fast_ll_dma_clear_tc2();
    if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        LL_DMA_ClearFlag_TE2(DMA1);
    }
}

static void fast_ll_dma_clear_ch3_flags(void)
{
    if (LL_DMA_IsActiveFlag_GI3(DMA1)) {
        LL_DMA_ClearFlag_GI3(DMA1);
    }
    if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);
    }
    if (LL_DMA_IsActiveFlag_TE3(DMA1)) {
        LL_DMA_ClearFlag_TE3(DMA1);
    }
}

static void fast_ll_dma_stop_min(encoder_spi_bus_t *bus)
{
    SPI_TypeDef *spi = fast_spi(bus);

    LL_SPI_DisableDMAReq_RX(spi);
    LL_SPI_DisableDMAReq_TX(spi);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    bus->busy = 0U;
}

static int fast_ll_dma_arm(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx, uint8_t set_rx_addr)
{
    SPI_TypeDef *spi = fast_spi(bus);

    if (set_rx_addr) {
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)rx);
    }
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)tx);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 1U);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1U);

    bus->busy = 1U;
    LL_SPI_EnableDMAReq_RX(spi);
    LL_SPI_EnableDMAReq_TX(spi);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    return 0;
}

static void fast_hw_stop(encoder_spi_bus_t *bus);

static void fast_hw_init(encoder_spi_bus_t *bus)
{
    SPI_TypeDef *spi = fast_spi(bus);
    const uint32_t dr = LL_SPI_DMA_GetRegAddr(spi);

    bus->busy = 0U;
    LL_SPI_Enable(spi);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, dr);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, dr);

    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

    fast_hw_stop(bus);
}

static void fast_hw_stop(encoder_spi_bus_t *bus)
{
    fast_ll_dma_stop_min(bus);
    fast_ll_dma_clear_ch2_flags_full();
    fast_ll_dma_clear_ch3_flags();
}

static int fast_start_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    if (tx == NULL || rx == NULL) {
        return -1;
    }
    if (bus->busy != 0U) {
        return -1;
    }

    fast_ll_dma_stop_min(bus);
    fast_ll_dma_clear_tc2();
    return fast_ll_dma_arm(bus, tx, rx, 1U);
}

static int fast_restart_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    if (tx == NULL || rx == NULL) {
        return -1;
    }
    if (bus->busy != 0U) {
        return -1;
    }

    fast_ll_dma_stop_min(bus);
    fast_ll_dma_clear_tc2();
    return fast_ll_dma_arm(bus, tx, rx, 0U);
}

static void fast_dma_isr(encoder_spi_bus_t *bus)
{
    if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        LL_DMA_ClearFlag_TE2(DMA1);
        fast_hw_stop(bus);
        if (bus->on_error != NULL) {
            bus->on_error(bus);
        }
        return;
    }

    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);
        fast_ll_dma_stop_min(bus);
        if (bus->on_rx_complete != NULL) {
            bus->on_rx_complete(bus);
        }
    }
}

const encoder_spi_bus_ops_t encoder_spi_bus_ops_spi1_fast = {
    .cs_low = fast_cs_low,
    .cs_high = fast_cs_high,
    .hw_init = fast_hw_init,
    .hw_stop = fast_hw_stop,
    .start_word = fast_start_word,
    .restart_word = fast_restart_word,
    .dma_isr = fast_dma_isr,
};
