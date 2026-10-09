/**
 * @file board_encoder_spi3.c
 * @date 2026-10-08
 * @brief SPI3 + PA15：KTH7823 DMA 异步读角（台架替代 AS5047）。
 *
 * 映射：SPI3 PB3/4/5，CS=PA15，DMA1 Ch6 RX / Ch7 TX。
 * Flash CS=PD2 须保持高。dma_isr 只进 Ch6。
 */

#include "board_encoder.h"

#include "hal_bridge.h"
#include "kth7823.h"
#include "spi.h"
#include "stm32g4xx_ll_dma.h"

encoder_spi_bus_t enc_kth_spi3_bus;
kth7823_ctx_t enc_kth_spi3_ctx;
KTH7823_HandleTypeDef enc_kth_spi3_hal;
encoder_t enc_kth_spi3;

/**
 * @brief DMA 收完转到编码器。
 */
static void enc_kth_spi3_bus_rx_complete(encoder_spi_bus_t *bus)
{
    (void)bus;
    encoder_on_spi_rx_complete(&enc_kth_spi3);
}

static void enc_kth_spi3_bus_error(encoder_spi_bus_t *bus)
{
    (void)bus;
    encoder_on_spi_error(&enc_kth_spi3);
}

/**
 * @brief 绑 SPI3/PA15/DMA，并异步初始化 KTH7823。
 */
void board_encoder_spi3_init(void)
{
    enc_kth_spi3_bus.ops = &encoder_spi_bus_ops_generic_g4;
    enc_kth_spi3_bus.spi = SPI3;
    enc_kth_spi3_bus.dma_ll_rx_ch = LL_DMA_CHANNEL_6;
    enc_kth_spi3_bus.dma_ll_tx_ch = LL_DMA_CHANNEL_7;
    enc_kth_spi3_bus.cs_port = GPIOA;
    enc_kth_spi3_bus.cs_pin = GPIO_PIN_15;
    enc_kth_spi3_bus.busy = 0U;
    enc_kth_spi3_bus.user_ctx = &enc_kth_spi3;
    enc_kth_spi3_bus.on_rx_complete = enc_kth_spi3_bus_rx_complete;
    enc_kth_spi3_bus.on_error = enc_kth_spi3_bus_error;

    KTH7823_Init(&enc_kth_spi3_hal, &hspi3, GPIOA, GPIO_PIN_15);

    enc_kth_spi3_ctx.hal = &enc_kth_spi3_hal;
    encoder_init(&enc_kth_spi3, &kth7823_encoder_driver, &enc_kth_spi3_ctx, &enc_kth_spi3_bus);
    (void)encoder_async_init(&enc_kth_spi3);
    encoder_spi_bus_cs_high(&enc_kth_spi3_bus);
}

/**
 * @brief SPI3 编码器 DMA RX 中断入口。
 */
void board_encoder_spi3_dma_isr(void)
{
    encoder_spi_bus_dma_isr(&enc_kth_spi3_bus);
}
