#include "gpio.h"
#include "spi.h"
#include "as5047.h"
#include "app_uart_dma_debug.h"
#if AS5047_SPI1_LL
#include "bsp_as5047_spi1_ll.h"
#endif

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f
AS5047_HandleTypeDef AS5047_spi1_PORT, AS5047_spi3_PORT;
// ?????????????
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
    uint16_t parity_bit_value = 0;
    while(data_2_cal != 0)
    {
        parity_bit_value ^= data_2_cal; 
        data_2_cal >>= 1;
    }
    return (parity_bit_value & 0x1); 
}

// SPI?????????????????SPI?????
uint16_t SPI_ReadWrite_OneByte(AS5047_HandleTypeDef *as5047, uint16_t _txdata)
{
    AS5047_CS_L(as5047);  // CS????
    
    uint16_t rxdata;
    if(HAL_SPI_TransmitReceive(as5047->hspi, (uint8_t *)&_txdata, 
                              (uint8_t *)&rxdata, 1,1000) != HAL_OK) {
        rxdata = 0;  // ?????????0
    }
    
    AS5047_CS_H(as5047);  // CS????
    return rxdata;
}

// AS5047???????
uint16_t AS5047_read(AS5047_HandleTypeDef *as5047, uint16_t add)
{
    uint16_t data;
    add |= 0x4000;  // ????? bit14 ??1
    
    // ????15??1?????????????Bit15??1
    if(Parity_bit_Calculate(add) == 1) {
        add = add | 0x8000;
    }
    
    SPI_ReadWrite_OneByte(as5047, add);  // ???????
    data = SPI_ReadWrite_OneByte(as5047, NOP | 0x4000);  // ??????????????
    
    data &= 0x3fff;  // ?14??????
    return data;
}

// ????????????
float AS5047_GetAngle(AS5047_HandleTypeDef *as5047)
{
    float angle_data = AS5047_read(as5047, ANGLEUNC);
    
    // ?????????
    float d_angle = angle_data - as5047->angle_data_prev;
    if(abs(d_angle) > (0.8f * AS5047_RESOLUTION)) {
        as5047->full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
    }
    as5047->angle_data_prev = angle_data;
    
    return (as5047->full_rotation_offset + (angle_data / (float)AS5047_RESOLUTION) * _2PI);
}

// AS5047?????????
void AS5047_Init(AS5047_HandleTypeDef *as5047, SPI_HandleTypeDef *hspi, 
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    as5047->hspi = hspi;
    as5047->cs_gpio_port = cs_port;
    as5047->cs_gpio_pin = cs_pin;
    as5047->angle_data_prev = 0;
    as5047->full_rotation_offset = 0;
    
    AS5047_CS_H(as5047);
}

/* --- SPI1 DMA bringup (AS5047_spi1_PORT only, 2 CS frames) --- */
typedef enum {
    AS5047_DMA_IDLE   = 0,
    AS5047_DMA_FRAME1 = 1,
    AS5047_DMA_FRAME2 = 2,
} as5047_dma_phase_t;

static uint16_t s_dma_tx_cmd;
static uint16_t s_dma_tx_nop;
static uint16_t s_dma_rx;
static volatile uint16_t s_dma_raw;
static volatile as5047_dma_phase_t s_dma_phase = AS5047_DMA_IDLE;
static AS5047_HandleTypeDef *s_dma_dev;
static volatile uint32_t s_dma_kick_cyccnt;
static volatile uint32_t s_dma_f1_cb_delta;

static uint32_t as5047_cyccnt(void)
{
    return *(volatile uint32_t *)&DWT->CYCCNT;
}

static void as5047_enc_update_total_delta(void)
{
    g_telem_dbg.enc_total_delta = g_telem_dbg.isr_delta + g_telem_dbg.enc_dma_cpu_delta;
}

static uint16_t as5047_build_read_cmd(uint16_t add)
{
    add |= 0x4000;
    if (Parity_bit_Calculate(add) == 1) {
        add |= 0x8000;
    }
    return add;
}

static int as5047_dma_start_word(AS5047_HandleTypeDef *as5047, uint16_t *tx)
{
#if AS5047_SPI1_LL
    (void)as5047;
    return bsp_as5047_spi1_ll_start_word(tx, &s_dma_rx);
#else
    if (as5047->hspi->State != HAL_SPI_STATE_READY) {
        return -1;
    }
    return (HAL_SPI_TransmitReceive_DMA(as5047->hspi,
                                         (uint8_t *)tx,
                                         (uint8_t *)&s_dma_rx,
                                         1) == HAL_OK) ? 0 : -1;
#endif
}

static void as5047_dma_abort(AS5047_HandleTypeDef *as5047)
{
    AS5047_CS_H(as5047);
#if AS5047_SPI1_LL
    bsp_as5047_spi1_ll_hw_stop();
#endif
    s_dma_phase = AS5047_DMA_IDLE;
    g_telem_dbg.enc_dma_busy = 0;
    g_telem_dbg.enc_dma_phase = AS5047_DMA_IDLE;
}

/** Start FRAME1 (CS + cmd DMA). Updates kick DWT stamps for enc_dma_seq_delta. */
static int as5047_dma_kick_frame1(AS5047_HandleTypeDef *as5047)
{
#if !AS5047_SPI1_LL
    if (as5047->hspi->State != HAL_SPI_STATE_READY) {
        g_telem_dbg.enc_kick_skip_spi++;
        return -1;
    }
#endif

    s_dma_dev = as5047;
    {
        uint32_t t0 = as5047_cyccnt();

        AS5047_CS_L(as5047);
        if (as5047_dma_start_word(as5047, &s_dma_tx_cmd) != 0) {
            as5047_dma_abort(as5047);
            g_telem_dbg.enc_err_cnt++;
            return -1;
        }
        g_telem_dbg.enc_dma_kick_delta = as5047_cyccnt() - t0;
        s_dma_kick_cyccnt = as5047_cyccnt();
    }

    s_dma_phase = AS5047_DMA_FRAME1;
    g_telem_dbg.enc_dma_busy = 1;
    g_telem_dbg.enc_dma_phase = AS5047_DMA_FRAME1;
    g_telem_dbg.enc_kick_cnt++;
    return 0;
}

static void as5047_dma_on_rx_complete(void)
{
    if (s_dma_dev == NULL) {
        return;
    }

#if AS5047_SPI1_LL
    g_telem_dbg.enc_spi_state = 0U;
#else
    g_telem_dbg.enc_spi_state = (uint8_t)s_dma_dev->hspi->State;
#endif

    if (s_dma_phase == AS5047_DMA_FRAME1) {
        uint32_t cb_t0 = as5047_cyccnt();

        AS5047_CS_H(s_dma_dev);
        g_telem_dbg.enc_rx_word0 = s_dma_rx;

        AS5047_CS_L(s_dma_dev);
        if (as5047_dma_start_word(s_dma_dev, &s_dma_tx_nop) != 0) {
            as5047_dma_abort(s_dma_dev);
            g_telem_dbg.enc_err_cnt++;
            return;
        }

        s_dma_f1_cb_delta = as5047_cyccnt() - cb_t0;
        g_telem_dbg.enc_dma_f1_cb_delta = s_dma_f1_cb_delta;

        s_dma_phase = AS5047_DMA_FRAME2;
        g_telem_dbg.enc_dma_phase = AS5047_DMA_FRAME2;
        return;
    }

    if (s_dma_phase == AS5047_DMA_FRAME2) {
        uint32_t cb_t0 = as5047_cyccnt();

        AS5047_CS_H(s_dma_dev);
        g_telem_dbg.enc_rx_word1 = s_dma_rx;
        s_dma_raw = s_dma_rx & 0x3FFF;
        g_telem_dbg.enc_raw = s_dma_raw;
        g_telem_dbg.enc_cplt_cnt++;

        g_telem_dbg.enc_dma_f2_cb_delta = as5047_cyccnt() - cb_t0;
        g_telem_dbg.enc_dma_cpu_delta = s_dma_f1_cb_delta + g_telem_dbg.enc_dma_f2_cb_delta;
        g_telem_dbg.enc_dma_seq_delta = as5047_cyccnt() - s_dma_kick_cyccnt;
        if (g_telem_dbg.enc_dma_seq_delta > g_telem_dbg.enc_dma_seq_delta_max) {
            g_telem_dbg.enc_dma_seq_delta_max = g_telem_dbg.enc_dma_seq_delta;
        }
        as5047_enc_update_total_delta();

        s_dma_phase = AS5047_DMA_IDLE;
        g_telem_dbg.enc_dma_busy = 0;
        g_telem_dbg.enc_dma_phase = AS5047_DMA_IDLE;
    }
}

void AS5047_DmaInit(AS5047_HandleTypeDef *as5047)
{
    s_dma_tx_cmd = as5047_build_read_cmd(ANGLEUNC);
    s_dma_tx_nop = NOP | 0x4000;
    s_dma_phase = AS5047_DMA_IDLE;
    s_dma_dev = as5047;
    s_dma_raw = AS5047_read(as5047, ANGLEUNC);

    g_telem_dbg.enc_dma_busy = 0;
    g_telem_dbg.enc_dma_phase = AS5047_DMA_IDLE;
    g_telem_dbg.enc_spi_state = (uint8_t)as5047->hspi->State;
    g_telem_dbg.enc_raw = s_dma_raw;
    g_telem_dbg.enc_rx_word1 = s_dma_raw;
    g_telem_dbg.enc_dma_kick_delta = 0u;
    g_telem_dbg.enc_dma_f1_cb_delta = 0u;
    g_telem_dbg.enc_dma_f2_cb_delta = 0u;
    g_telem_dbg.enc_dma_cpu_delta = 0u;
    g_telem_dbg.enc_dma_seq_delta = 0u;
    g_telem_dbg.enc_dma_seq_delta_max = 0u;
    g_telem_dbg.enc_total_delta = 0u;
    g_telem_dbg.enc_chain_kick_cnt = 0u;

#if AS5047_SPI1_LL
    bsp_as5047_spi1_ll_init();
#endif

    if (as5047_dma_kick_frame1(as5047) == 0) {
        g_telem_dbg.enc_chain_kick_cnt++;
    }
}

uint16_t AS5047_GetRaw(void)
{
    return s_dma_raw;
}

void AS5047_DmaKick(AS5047_HandleTypeDef *as5047)
{
    g_telem_dbg.enc_spi_state = (uint8_t)as5047->hspi->State;
    g_telem_dbg.enc_dma_busy = (s_dma_phase != AS5047_DMA_IDLE) ? 1U : 0U;
    g_telem_dbg.enc_dma_phase = (uint8_t)s_dma_phase;

    if (s_dma_phase != AS5047_DMA_IDLE) {
        g_telem_dbg.enc_kick_skip_busy++;
        return;
    }

    (void)as5047_dma_kick_frame1(as5047);
}

#if AS5047_SPI1_LL
void AS5047_Spi1LL_OnRxComplete(void)
{
    as5047_dma_on_rx_complete();
}

void AS5047_Spi1LL_OnError(void)
{
    if (s_dma_dev == NULL) {
        return;
    }

    as5047_dma_abort(s_dma_dev);
    g_telem_dbg.enc_spi_state = 0U;
    g_telem_dbg.enc_err_cnt++;
}
#else
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1 || s_dma_dev == NULL) {
        return;
    }

    as5047_dma_on_rx_complete();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1 || s_dma_dev == NULL) {
        return;
    }

    as5047_dma_abort(s_dma_dev);
    g_telem_dbg.enc_spi_state = (uint8_t)hspi->State;
    g_telem_dbg.enc_err_cnt++;
}
#endif