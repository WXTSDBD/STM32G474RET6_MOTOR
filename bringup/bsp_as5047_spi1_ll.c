#include "bsp_as5047_spi1_ll.h"
#include "main.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_bus.h"

void AS5047_Spi1LL_OnRxComplete(void);
void AS5047_Spi1LL_OnError(void);

static volatile uint8_t s_ll_busy;

static void ll_dma_clear_ch2_flags(void)
{
    if (LL_DMA_IsActiveFlag_GI2(DMA1)) {
        LL_DMA_ClearFlag_GI2(DMA1);
    }
    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);
    }
    if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        LL_DMA_ClearFlag_TE2(DMA1);
    }
}

static void ll_dma_clear_ch3_flags(void)
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

void bsp_as5047_spi1_ll_init(void)
{
    s_ll_busy = 0U;

    LL_SPI_Enable(SPI1);

    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);

    bsp_as5047_spi1_ll_hw_stop();
}

void bsp_as5047_spi1_ll_hw_stop(void)
{
    LL_SPI_DisableDMAReq_RX(SPI1);
    LL_SPI_DisableDMAReq_TX(SPI1);

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

    ll_dma_clear_ch2_flags();
    ll_dma_clear_ch3_flags();

    s_ll_busy = 0U;
}

int bsp_as5047_spi1_ll_start_word(const uint16_t *tx, uint16_t *rx)
{
    const uint32_t dr = LL_SPI_DMA_GetRegAddr(SPI1);

    if (tx == NULL || rx == NULL) {
        return -1;
    }
    if (s_ll_busy != 0U) {
        return -1;
    }

    LL_SPI_DisableDMAReq_RX(SPI1);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    ll_dma_clear_ch2_flags();
    ll_dma_clear_ch3_flags();

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)rx);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, dr);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 1U);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)tx);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, dr);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1U);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

    s_ll_busy = 1U;

    LL_SPI_EnableDMAReq_RX(SPI1);
    LL_SPI_EnableDMAReq_TX(SPI1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

    return 0;
}

void bsp_as5047_spi1_ll_dma1_ch2_isr(void)
{
    if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        LL_DMA_ClearFlag_TE2(DMA1);
        ll_dma_clear_ch2_flags();
        bsp_as5047_spi1_ll_hw_stop();
        AS5047_Spi1LL_OnError();
        return;
    }

    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);
        ll_dma_clear_ch2_flags();
        bsp_as5047_spi1_ll_hw_stop();
        AS5047_Spi1LL_OnRxComplete();
    }
}
