#include "board_encoder.h"

#include "hal_bridge.h"
#include "spi.h"
#include "stm32g4xx_ll_dma.h"

encoder_spi_bus_t enc_m1_bus;
as5047_ctx_t enc_m1_as5047;
encoder_t enc_m1;

encoder_t enc_m2;

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
}

void board_encoder_m1_dma_isr(void)
{
    encoder_spi_bus_dma_isr(&enc_m1_bus);
}
