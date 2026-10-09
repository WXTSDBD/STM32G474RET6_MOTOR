/**
 * @file board_encoder_m1.c
 * @date 2026-10-06
 * @brief SPI1+PA4：AS5047 DMA 路径；台架默认可经 M1_ENCODER_SRC 切到 SPI3/KTH。
 *
 * board_encoder_m1_setup() 按宏选 SPI1/AS5047 或 SPI3/KTH7823。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "board_encoder.h"

#include "as5047.h"
#include "bringup_bench.h"
#include "hal_bridge.h"
#include "spi.h"
#include "stm32g4xx_ll_dma.h"

encoder_spi_bus_t enc_m1_bus;
as5047_ctx_t enc_m1_as5047;
encoder_t enc_m1;

encoder_t enc_m2;

/**
 * @brief DMA 收完转到编码器。
 */
static void enc_m1_bus_rx_complete(encoder_spi_bus_t *bus)
{
    (void)bus;
    encoder_on_spi_rx_complete(&enc_m1);
}

static void enc_m1_bus_error(encoder_spi_bus_t *bus)
{
    (void)bus;
    encoder_on_spi_error(&enc_m1);
}

/**
 * @brief 绑 SPI1/PA4/DMA，并异步初始化 AS5047。
 */
void board_encoder_m1_init(void)
{
    enc_m1_bus.ops = &encoder_spi_bus_ops_spi1_fast;
    enc_m1_bus.spi = SPI1;
    enc_m1_bus.dma_ll_rx_ch = LL_DMA_CHANNEL_2;
    enc_m1_bus.dma_ll_tx_ch = LL_DMA_CHANNEL_3;
    enc_m1_bus.cs_port = GPIOA;
    enc_m1_bus.cs_pin = GPIO_PIN_4;
    enc_m1_bus.busy = 0U;
    enc_m1_bus.user_ctx = &enc_m1;
    enc_m1_bus.on_rx_complete = enc_m1_bus_rx_complete;
    enc_m1_bus.on_error = enc_m1_bus_error;

    AS5047_Init(&AS5047_spi1_PORT, &hspi1, GPIOA, GPIO_PIN_4);

    enc_m1_as5047.hal = &AS5047_spi1_PORT;
    encoder_init(&enc_m1, &as5047_encoder_driver, &enc_m1_as5047, &enc_m1_bus);
    (void)encoder_async_init(&enc_m1);
    /* Cube 把 CS 初值拉低；空闲须释放，否则 SPI 易卡在半帧 */
    encoder_spi_bus_cs_high(&enc_m1_bus);
}

/**
 * @brief M1 编码器 DMA 中断入口。
 */
void board_encoder_m1_dma_isr(void)
{
    encoder_spi_bus_dma_isr(&enc_m1_bus);
}

/**
 * @brief 按台架宏初始化 M1 角源并返回实例指针。
 */
encoder_t *board_encoder_m1_setup(void)
{
#if (M1_ENCODER_SRC == M1_ENCODER_SRC_KTH7823_SPI3)
    board_encoder_spi3_init();
    return &enc_kth_spi3;
#else
    board_encoder_m1_init();
    return &enc_m1;
#endif
}
