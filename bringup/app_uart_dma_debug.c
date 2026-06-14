/**
 * @file app_uart_dma_debug.c
 * @brief 双缓冲 + DMA 发送（Step 6a：ISR 写缓冲，任务发 DMA）
 *
 * 数据路径：
 *   TIM1 ISR：telem_bringup_tick() — 仅写双缓冲
 *   RTOS 任务：telem_bringup_try_send() — READY 时 DMA 发送
 *   TxCplt：SENDING → UNLOCKED
 *
 * 测试通道（JustFloat ×4）：
 *   ch0=foc_iq  ch1=foc_id  ch2=as5047_spi1.raw  ch3=as5047_spi1.add
 *
 * AS5047 DMA 耗时（g_telem_dbg，与 isr_delta 互补）：
 *   enc_dma_kick_delta  FRAME1 kick (LL DMA chain or HAL DmaKick)
 *   enc_dma_f1_cb_delta / enc_dma_f2_cb_delta  两次 SPI DMA 回调 CPU
 *   enc_dma_cpu_delta     f1_cb + f2_cb
 *   enc_dma_seq_delta     kick→FRAME2 完成（含硬件等待，非纯 CPU）
 *   enc_total_delta       isr_delta + enc_dma_cpu_delta（整拍参考）
 *
 * VOFA+：6000000，JustFloat，△t ≈ D / 20000 秒（TIM1 20kHz 基准）
 */

#include "app_uart_dma_debug.h"
#include "cmsis_os.h"
#include "usart.h"
#include "bsp_dwt.h"
#include "main.h"
#include "FOC_CAL.h"
#include "as5047.h"
#include "encoder_spi_bus.h"
#include <string.h>

#define TELEM_BRINGUP_K           4u
#define TELEM_BRINGUP_INCLUDE_SEQ 0u
#define TELEM_CPU_MHZ             160u

/** 单缓冲 4KB，双缓冲共 8KB SRAM */
#define TELEM_BUF_BYTES           4096u

/**
 * TIM1 20kHz 下每 D 次 tick 写 1 个小帧。
 * D=100 → 200Hz 写帧，满包 204 帧 ≈ 1.0s
 */
#define TELEM_BRINGUP_DECIMATION  100u

#if TELEM_BRINGUP_INCLUDE_SEQ
#define TELEM_SMALL_FRAME_BYTES   (4u + TELEM_BRINGUP_K * 4u + 4u)
#else
#define TELEM_SMALL_FRAME_BYTES   (TELEM_BRINGUP_K * 4u + 4u)
#endif

#if TELEM_BRINGUP_INCLUDE_SEQ
#define TELEM_CH1_BYTE_OFF        8u
#define TELEM_CH2_BYTE_OFF        12u
#else
#define TELEM_CH1_BYTE_OFF        4u
#define TELEM_CH2_BYTE_OFF        8u
#endif

static const uint8_t s_justfloat_tail[4] = {0x00u, 0x00u, 0x80u, 0x7fu};

typedef enum {
    TELEM_BUF_UNLOCKED = 0,
    TELEM_BUF_LOCKED,
    TELEM_BUF_READY,
    TELEM_BUF_SENDING
} telem_buf_state_t;

typedef struct {
    uint8_t            data[TELEM_BUF_BYTES];
    uint16_t           used_bytes;
    volatile telem_buf_state_t state;
} telem_buf_t;

static telem_buf_t s_bufs[2];
static telem_buf_t *s_write_buf;
static telem_buf_t *s_sending_buf;

#if TELEM_BRINGUP_INCLUDE_SEQ
static uint32_t s_seq;
#endif

static uint32_t s_decim_cnt;

telem_dbg_t g_telem_dbg;

/** 调试：最近一次 ch2_wire（isr_delta） */
volatile uint32_t time_cnt;

static uint32_t s_prof_kick_cyccnt;
static uint32_t s_prof_f1_delta;

static void telem_enc_profile_cb(const encoder_t *e, const enc_profile_event_t *ev)
{
    const as5047_ctx_t *ctx = (const as5047_ctx_t *)e->chip_ctx;

    (void)e;
    if (ctx == NULL) {
        return;
    }

    g_telem_dbg.enc_spi_state = encoder_spi_bus_is_busy(e->bus);

    switch (ev->ev) {
    case ENC_EVT_KICK_DONE:
        g_telem_dbg.enc_dma_kick_delta = ev->aux;
        s_prof_kick_cyccnt = ev->cyccnt;
        g_telem_dbg.enc_kick_cnt++;
        g_telem_dbg.enc_dma_busy = 1U;
        g_telem_dbg.enc_dma_phase = AS5047_PHASE_FRAME1;
        break;
    case ENC_EVT_F1_DONE:
        s_prof_f1_delta = ev->aux;
        g_telem_dbg.enc_dma_f1_cb_delta = ev->aux;
        g_telem_dbg.enc_rx_word0 = ctx->rx_buf;
        g_telem_dbg.enc_dma_phase = AS5047_PHASE_FRAME2;
        break;
    case ENC_EVT_F2_DONE:
        g_telem_dbg.enc_dma_f2_cb_delta = ev->aux;
        g_telem_dbg.enc_dma_cpu_delta = s_prof_f1_delta + ev->aux;
        g_telem_dbg.enc_dma_seq_delta = ev->cyccnt - s_prof_kick_cyccnt;
        if (g_telem_dbg.enc_dma_seq_delta > g_telem_dbg.enc_dma_seq_delta_max) {
            g_telem_dbg.enc_dma_seq_delta_max = g_telem_dbg.enc_dma_seq_delta;
        }
        g_telem_dbg.enc_total_delta = g_telem_dbg.isr_delta + g_telem_dbg.enc_dma_cpu_delta;
        g_telem_dbg.enc_rx_word1 = ctx->rx_buf;
        g_telem_dbg.enc_raw = ctx->raw;
        g_telem_dbg.enc_cplt_cnt++;
        g_telem_dbg.enc_dma_busy = 0U;
        g_telem_dbg.enc_dma_phase = AS5047_PHASE_IDLE;
        break;
    case ENC_EVT_KICK_SKIP_BUSY:
        g_telem_dbg.enc_kick_skip_busy++;
        g_telem_dbg.enc_dma_busy = (ctx->phase != AS5047_PHASE_IDLE) ? 1U : 0U;
        g_telem_dbg.enc_dma_phase = ctx->phase;
        break;
    case ENC_EVT_ERROR:
        g_telem_dbg.enc_err_cnt++;
        g_telem_dbg.enc_dma_busy = 0U;
        g_telem_dbg.enc_dma_phase = AS5047_PHASE_IDLE;
        break;
    default:
        break;
    }
}

void telem_encoder_profile_bind(encoder_t *e)
{
    encoder_set_profile_cb(e, telem_enc_profile_cb);
}

static uint8_t telem_dma_busy(void)
{
    return (hlpuart1.gState == HAL_UART_STATE_BUSY_TX) ? 1u : 0u;
}

static telem_buf_t *telem_find_buf(telem_buf_state_t want)
{
    uint32_t i;

    for (i = 0u; i < 2u; i++) {
        if (s_bufs[i].state == want) {
            return &s_bufs[i];
        }
    }
    return NULL;
}

static int telem_acquire_write_buf(void)
{
    telem_buf_t *buf;

    if (s_write_buf != NULL) {
        return 1;
    }

    buf = telem_find_buf(TELEM_BUF_UNLOCKED);
    if (buf == NULL) {
        return 0;
    }

    buf->used_bytes = 0u;
    buf->state = TELEM_BUF_LOCKED;
    s_write_buf = buf;
    return 1;
}

static void telem_refresh_buf_snapshot(void)
{
    g_telem_dbg.buf0_state = (uint8_t)s_bufs[0].state;
    g_telem_dbg.buf1_state = (uint8_t)s_bufs[1].state;
    g_telem_dbg.buf0_used = s_bufs[0].used_bytes;
    g_telem_dbg.buf1_used = s_bufs[1].used_bytes;
    g_telem_dbg.write_buf_active = (s_write_buf != NULL) ? 1u : 0u;
    g_telem_dbg.uart_gstate = (uint8_t)hlpuart1.gState;
    g_telem_dbg.uart_error = hlpuart1.ErrorCode;
}

static void telem_seal_write_buf_ready(void)
{
    if (s_write_buf == NULL) {
        return;
    }
    s_write_buf->state = TELEM_BUF_READY;
    s_write_buf = NULL;
    g_telem_dbg.seal_cnt++;
}

static void telem_write_small_frame(telem_buf_t *buf, uint16_t offset)
{
    float vals[4];
    uint8_t *p;

    vals[0] = dbg.foc_iq;
    vals[1] = dbg.foc_id;
    vals[2] = (float)as5047_spi1.raw;
    vals[3] = as5047_spi1.add;

    p = &buf->data[offset];

#if TELEM_BRINGUP_INCLUDE_SEQ
    {
        uint32_t seq = s_seq++;
        memcpy(p, &seq, sizeof(seq));
        p += sizeof(seq);
    }
#endif

    memcpy(p, vals, sizeof(vals));
    p += sizeof(vals);
    memcpy(p, s_justfloat_tail, sizeof(s_justfloat_tail));
}

void telem_bringup_tick(void)
{
    telem_buf_t *buf;

    g_telem_dbg.tick_total++;
    s_decim_cnt++;
    if (s_decim_cnt < TELEM_BRINGUP_DECIMATION) {
        g_telem_dbg.tick_decim_skip++;
        return;
    }
    s_decim_cnt = 0u;

    if (!telem_acquire_write_buf()) {
        g_telem_dbg.acquire_fail++;
        return;
    }

    buf = s_write_buf;

    if ((uint32_t)buf->used_bytes + TELEM_SMALL_FRAME_BYTES > TELEM_BUF_BYTES) {
        telem_seal_write_buf_ready();
        if (!telem_acquire_write_buf()) {
            g_telem_dbg.acquire_fail++;
            return;
        }
        buf = s_write_buf;
    }

    {
        uint16_t off = buf->used_bytes;
        uint32_t ch1_raw = g_telem_dbg.cyccnt_end;
        uint32_t ch2_raw = g_telem_dbg.isr_delta;

        telem_write_small_frame(buf, off);
        g_telem_dbg.ch1_wire = ch1_raw;
        g_telem_dbg.ch2_wire = ch2_raw;
        time_cnt = ch2_raw;
    }
    buf->used_bytes = (uint16_t)(buf->used_bytes + TELEM_SMALL_FRAME_BYTES);

    if ((uint32_t)buf->used_bytes + TELEM_SMALL_FRAME_BYTES > TELEM_BUF_BYTES) {
        telem_seal_write_buf_ready();
    }
    g_telem_dbg.tick_frame_ok++;
}

void telem_bringup_try_send(void)
{
    telem_buf_t *buf;

    g_telem_dbg.try_send_calls++;
    telem_refresh_buf_snapshot();

    if (telem_dma_busy()) {
        g_telem_dbg.dma_busy_skip++;
        return;
    }

    buf = telem_find_buf(TELEM_BUF_READY);
    if (buf == NULL || buf->used_bytes == 0u) {
        g_telem_dbg.no_ready_skip++;
        return;
    }

    if (HAL_UART_Transmit_DMA(&hlpuart1, buf->data, buf->used_bytes) != HAL_OK) {
        g_telem_dbg.dma_start_fail++;
        return;
    }

    g_telem_dbg.dma_start_ok++;
    g_telem_dbg.last_dma_bytes = buf->used_bytes;
    buf->state = TELEM_BUF_SENDING;
    s_sending_buf = buf;
    telem_refresh_buf_snapshot();
}

static void telem_bringup_on_dma_done(void)
{
    if (s_sending_buf == NULL) {
        return;
    }

    s_sending_buf->used_bytes = 0u;
    s_sending_buf->state = TELEM_BUF_UNLOCKED;
    s_sending_buf = NULL;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1) {
        g_telem_dbg.tx_cplt_cnt++;
        telem_bringup_on_dma_done();
    }
}

void telem_bringup_init(void)
{
    uint32_t i;

    DWT_Init(TELEM_CPU_MHZ);

    for (i = 0u; i < 2u; i++) {
        s_bufs[i].used_bytes = 0u;
        s_bufs[i].state = TELEM_BUF_UNLOCKED;
    }

    s_write_buf = NULL;
    s_sending_buf = NULL;
    s_decim_cnt = 0u;
    g_telem_dbg.isr_t0 = 0u;
    g_telem_dbg.isr_t1 = 0u;
    g_telem_dbg.cyccnt_end = 0u;
    g_telem_dbg.isr_delta = 0u;
    g_telem_dbg.ch1_wire = 0u;
    g_telem_dbg.ch2_wire = 0u;
    g_telem_dbg.tick_total = 0u;
    g_telem_dbg.tick_decim_skip = 0u;
    g_telem_dbg.tick_frame_ok = 0u;
    g_telem_dbg.acquire_fail = 0u;
    g_telem_dbg.seal_cnt = 0u;
    g_telem_dbg.uart_task_loops = 0u;
    g_telem_dbg.try_send_calls = 0u;
    g_telem_dbg.dma_busy_skip = 0u;
    g_telem_dbg.no_ready_skip = 0u;
    g_telem_dbg.dma_start_ok = 0u;
    g_telem_dbg.dma_start_fail = 0u;
    g_telem_dbg.tx_cplt_cnt = 0u;
    g_telem_dbg.buf0_state = 0u;
    g_telem_dbg.buf1_state = 0u;
    g_telem_dbg.buf0_used = 0u;
    g_telem_dbg.buf1_used = 0u;
    g_telem_dbg.write_buf_active = 0u;
    g_telem_dbg.uart_gstate = 0u;
    g_telem_dbg.uart_error = 0u;
    g_telem_dbg.last_dma_bytes = 0u;
    g_telem_dbg.enc_dma_kick_delta = 0u;
    g_telem_dbg.enc_dma_f1_cb_delta = 0u;
    g_telem_dbg.enc_dma_f2_cb_delta = 0u;
    g_telem_dbg.enc_dma_cpu_delta = 0u;
    g_telem_dbg.enc_dma_seq_delta = 0u;
    g_telem_dbg.enc_dma_seq_delta_max = 0u;
    g_telem_dbg.enc_total_delta = 0u;
    g_telem_dbg.enc_chain_kick_cnt = 0u;
    time_cnt = 0u;
#if TELEM_BRINGUP_INCLUDE_SEQ
    s_seq = 0u;
#endif
}

void UART_DMA_DEBUG_TASK(void *argument)
{
    (void)argument;

    for (;;) {
        g_telem_dbg.uart_task_loops++;
        telem_bringup_try_send();
        osDelay(1);
    }
}
