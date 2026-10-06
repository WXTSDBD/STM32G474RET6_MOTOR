/**
 * @file encoder_spi_bus_generic_g4.c
 * @date 2026-10-06
 * @brief G4 通用 DMA 后端。给还没绑快速通道的轴。

 *
 * dma_isr 只进该总线对应 DMA 中断。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "encoder_spi_bus.h"

#include <stddef.h>

#include "hal_bridge.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"

static uint32_t ll_dma_is_active_tc(DMA_TypeDef *dma, uint32_t ch)
{
    switch (ch) {
    case LL_DMA_CHANNEL_1: return LL_DMA_IsActiveFlag_TC1(dma);
    case LL_DMA_CHANNEL_2: return LL_DMA_IsActiveFlag_TC2(dma);
    case LL_DMA_CHANNEL_3: return LL_DMA_IsActiveFlag_TC3(dma);
    case LL_DMA_CHANNEL_4: return LL_DMA_IsActiveFlag_TC4(dma);
    case LL_DMA_CHANNEL_5: return LL_DMA_IsActiveFlag_TC5(dma);
    case LL_DMA_CHANNEL_6: return LL_DMA_IsActiveFlag_TC6(dma);
    case LL_DMA_CHANNEL_7: return LL_DMA_IsActiveFlag_TC7(dma);
    case LL_DMA_CHANNEL_8: return LL_DMA_IsActiveFlag_TC8(dma);
    default: return 0U;
    }
}

static void ll_dma_clear_tc_flag(DMA_TypeDef *dma, uint32_t ch)
{
    switch (ch) {
    case LL_DMA_CHANNEL_1: LL_DMA_ClearFlag_TC1(dma); break;
    case LL_DMA_CHANNEL_2: LL_DMA_ClearFlag_TC2(dma); break;
    case LL_DMA_CHANNEL_3: LL_DMA_ClearFlag_TC3(dma); break;
    case LL_DMA_CHANNEL_4: LL_DMA_ClearFlag_TC4(dma); break;
    case LL_DMA_CHANNEL_5: LL_DMA_ClearFlag_TC5(dma); break;
    case LL_DMA_CHANNEL_6: LL_DMA_ClearFlag_TC6(dma); break;
    case LL_DMA_CHANNEL_7: LL_DMA_ClearFlag_TC7(dma); break;
    case LL_DMA_CHANNEL_8: LL_DMA_ClearFlag_TC8(dma); break;
    default: break;
    }
}

static uint32_t ll_dma_is_active_te(DMA_TypeDef *dma, uint32_t ch)
{
    switch (ch) {
    case LL_DMA_CHANNEL_1: return LL_DMA_IsActiveFlag_TE1(dma);
    case LL_DMA_CHANNEL_2: return LL_DMA_IsActiveFlag_TE2(dma);
    case LL_DMA_CHANNEL_3: return LL_DMA_IsActiveFlag_TE3(dma);
    case LL_DMA_CHANNEL_4: return LL_DMA_IsActiveFlag_TE4(dma);
    case LL_DMA_CHANNEL_5: return LL_DMA_IsActiveFlag_TE5(dma);
    case LL_DMA_CHANNEL_6: return LL_DMA_IsActiveFlag_TE6(dma);
    case LL_DMA_CHANNEL_7: return LL_DMA_IsActiveFlag_TE7(dma);
    case LL_DMA_CHANNEL_8: return LL_DMA_IsActiveFlag_TE8(dma);
    default: return 0U;
    }
}

static void ll_dma_clear_te_flag(DMA_TypeDef *dma, uint32_t ch)
{
    switch (ch) {
    case LL_DMA_CHANNEL_1: LL_DMA_ClearFlag_TE1(dma); break;
    case LL_DMA_CHANNEL_2: LL_DMA_ClearFlag_TE2(dma); break;
    case LL_DMA_CHANNEL_3: LL_DMA_ClearFlag_TE3(dma); break;
    case LL_DMA_CHANNEL_4: LL_DMA_ClearFlag_TE4(dma); break;
    case LL_DMA_CHANNEL_5: LL_DMA_ClearFlag_TE5(dma); break;
    case LL_DMA_CHANNEL_6: LL_DMA_ClearFlag_TE6(dma); break;
    case LL_DMA_CHANNEL_7: LL_DMA_ClearFlag_TE7(dma); break;
    case LL_DMA_CHANNEL_8: LL_DMA_ClearFlag_TE8(dma); break;
    default: break;
    }
}

static uint32_t ll_dma_is_active_gi(DMA_TypeDef *dma, uint32_t ch)
{
    switch (ch) {
    case LL_DMA_CHANNEL_1: return LL_DMA_IsActiveFlag_GI1(dma);
    case LL_DMA_CHANNEL_2: return LL_DMA_IsActiveFlag_GI2(dma);
    case LL_DMA_CHANNEL_3: return LL_DMA_IsActiveFlag_GI3(dma);
    case LL_DMA_CHANNEL_4: return LL_DMA_IsActiveFlag_GI4(dma);
    case LL_DMA_CHANNEL_5: return LL_DMA_IsActiveFlag_GI5(dma);
    case LL_DMA_CHANNEL_6: return LL_DMA_IsActiveFlag_GI6(dma);
    case LL_DMA_CHANNEL_7: return LL_DMA_IsActiveFlag_GI7(dma);
    case LL_DMA_CHANNEL_8: return LL_DMA_IsActiveFlag_GI8(dma);
    default: return 0U;
    }
}

static void ll_dma_clear_gi_flag(DMA_TypeDef *dma, uint32_t ch)
{
    switch (ch) {
    case LL_DMA_CHANNEL_1: LL_DMA_ClearFlag_GI1(dma); break;
    case LL_DMA_CHANNEL_2: LL_DMA_ClearFlag_GI2(dma); break;
    case LL_DMA_CHANNEL_3: LL_DMA_ClearFlag_GI3(dma); break;
    case LL_DMA_CHANNEL_4: LL_DMA_ClearFlag_GI4(dma); break;
    case LL_DMA_CHANNEL_5: LL_DMA_ClearFlag_GI5(dma); break;
    case LL_DMA_CHANNEL_6: LL_DMA_ClearFlag_GI6(dma); break;
    case LL_DMA_CHANNEL_7: LL_DMA_ClearFlag_GI7(dma); break;
    case LL_DMA_CHANNEL_8: LL_DMA_ClearFlag_GI8(dma); break;
    default: break;
    }
}

static void gen_ll_dma_clear_tc(encoder_spi_bus_t *bus)
{
    if (ll_dma_is_active_tc(DMA1, bus->dma_ll_rx_ch)) {
        ll_dma_clear_tc_flag(DMA1, bus->dma_ll_rx_ch);
    }
    if (ll_dma_is_active_gi(DMA1, bus->dma_ll_rx_ch)) {
        ll_dma_clear_gi_flag(DMA1, bus->dma_ll_rx_ch);
    }
}

static void gen_ll_dma_clear_rx_flags_full(encoder_spi_bus_t *bus)
{
    gen_ll_dma_clear_tc(bus);
    if (ll_dma_is_active_te(DMA1, bus->dma_ll_rx_ch)) {
        ll_dma_clear_te_flag(DMA1, bus->dma_ll_rx_ch);
    }
}

static void gen_ll_dma_clear_tx_flags(encoder_spi_bus_t *bus)
{
    if (ll_dma_is_active_gi(DMA1, bus->dma_ll_tx_ch)) {
        ll_dma_clear_gi_flag(DMA1, bus->dma_ll_tx_ch);
    }
    if (ll_dma_is_active_tc(DMA1, bus->dma_ll_tx_ch)) {
        ll_dma_clear_tc_flag(DMA1, bus->dma_ll_tx_ch);
    }
    if (ll_dma_is_active_te(DMA1, bus->dma_ll_tx_ch)) {
        ll_dma_clear_te_flag(DMA1, bus->dma_ll_tx_ch);
    }
}

static void gen_ll_dma_stop_min(encoder_spi_bus_t *bus)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)bus->spi;

    LL_SPI_DisableDMAReq_RX(spi);
    LL_SPI_DisableDMAReq_TX(spi);
    LL_DMA_DisableChannel(DMA1, bus->dma_ll_rx_ch);
    LL_DMA_DisableChannel(DMA1, bus->dma_ll_tx_ch);
    bus->busy = 0U;
}

static int gen_ll_dma_arm(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx, uint8_t set_rx_addr)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)bus->spi;

    if (set_rx_addr) {
        LL_DMA_SetMemoryAddress(DMA1, bus->dma_ll_rx_ch, (uint32_t)rx);
    }
    LL_DMA_SetMemoryAddress(DMA1, bus->dma_ll_tx_ch, (uint32_t)tx);
    LL_DMA_SetDataLength(DMA1, bus->dma_ll_rx_ch, 1U);
    LL_DMA_SetDataLength(DMA1, bus->dma_ll_tx_ch, 1U);

    bus->busy = 1U;
    LL_SPI_EnableDMAReq_RX(spi);
    LL_SPI_EnableDMAReq_TX(spi);
    LL_DMA_EnableChannel(DMA1, bus->dma_ll_tx_ch);
    LL_DMA_EnableChannel(DMA1, bus->dma_ll_rx_ch);
    return 0;
}

static void gen_cs_low(encoder_spi_bus_t *bus)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)bus->cs_port;

    port->BSRR = ((uint32_t)bus->cs_pin << 16U);
}

static void gen_cs_high(encoder_spi_bus_t *bus)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)bus->cs_port;

    port->BSRR = (uint32_t)bus->cs_pin;
}

static void gen_hw_stop(encoder_spi_bus_t *bus);

static void gen_hw_init(encoder_spi_bus_t *bus)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)bus->spi;
    const uint32_t dr = LL_SPI_DMA_GetRegAddr(spi);

    bus->busy = 0U;
    LL_SPI_Enable(spi);

    LL_DMA_SetPeriphAddress(DMA1, bus->dma_ll_rx_ch, dr);
    LL_DMA_SetPeriphAddress(DMA1, bus->dma_ll_tx_ch, dr);

    LL_DMA_DisableIT_TC(DMA1, bus->dma_ll_tx_ch);
    LL_DMA_DisableIT_TE(DMA1, bus->dma_ll_tx_ch);
    LL_DMA_DisableIT_HT(DMA1, bus->dma_ll_tx_ch);

    if (bus->dma_ll_tx_ch == LL_DMA_CHANNEL_3) {
        HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    }

    LL_DMA_EnableIT_TC(DMA1, bus->dma_ll_rx_ch);
    LL_DMA_EnableIT_TE(DMA1, bus->dma_ll_rx_ch);

    gen_hw_stop(bus);
}

static void gen_hw_stop(encoder_spi_bus_t *bus)
{
    gen_ll_dma_stop_min(bus);
    gen_ll_dma_clear_rx_flags_full(bus);
    gen_ll_dma_clear_tx_flags(bus);
}

static int gen_start_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    if (tx == NULL || rx == NULL) {
        return -1;
    }
    if (bus->busy != 0U) {
        return -1;
    }

    gen_ll_dma_stop_min(bus);
    gen_ll_dma_clear_tc(bus);
    return gen_ll_dma_arm(bus, tx, rx, 1U);
}

static int gen_restart_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx)
{
    if (tx == NULL || rx == NULL) {
        return -1;
    }
    if (bus->busy != 0U) {
        return -1;
    }

    gen_ll_dma_stop_min(bus);
    gen_ll_dma_clear_tc(bus);
    return gen_ll_dma_arm(bus, tx, rx, 0U);
}

static void gen_dma_isr(encoder_spi_bus_t *bus)
{
    if (ll_dma_is_active_te(DMA1, bus->dma_ll_rx_ch)) {
        ll_dma_clear_te_flag(DMA1, bus->dma_ll_rx_ch);
        gen_hw_stop(bus);
        if (bus->on_error != NULL) {
            bus->on_error(bus);
        }
        return;
    }

    if (ll_dma_is_active_tc(DMA1, bus->dma_ll_rx_ch)) {
        ll_dma_clear_tc_flag(DMA1, bus->dma_ll_rx_ch);
        gen_ll_dma_stop_min(bus);
        if (bus->on_rx_complete != NULL) {
            bus->on_rx_complete(bus);
        }
    }
}

const encoder_spi_bus_ops_t encoder_spi_bus_ops_generic_g4 = {
    .cs_low = gen_cs_low,
    .cs_high = gen_cs_high,
    .hw_init = gen_hw_init,
    .hw_stop = gen_hw_stop,
    .start_word = gen_start_word,
    .restart_word = gen_restart_word,
    .dma_isr = gen_dma_isr,
};
