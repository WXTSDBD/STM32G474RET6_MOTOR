/**
 * @file app_uart_dma_debug.c
 * @brief 双缓冲 + DMA 发送（Step 6a：ISR 写缓冲，任务发 DMA）
 *
 * 数据路径：
 *   TIM1 ISR：telem_bringup_tick() — 仅写双缓冲
 *   RTOS 任务：telem_bringup_try_send() — READY 时 DMA 发送
 *   TxCplt：SENDING → UNLOCKED
 *
 * 测试通道（CH_COUNT=4，ch1/ch2 为 uint32 原样小端，VOFA 勿当 float 看）：
 *   ch0=(float)cnt  ch1=上一拍 cyccnt_end  ch2=上一拍 isr_delta  ch3=4.0
 *
 * VOFA+：6000000，JustFloat，△t ≈ D / 20000 秒（TIM1 20kHz 基准）
 */

#include "app_uart_dma_debug.h"
#include "cmsis_os.h"
#include "usart.h"
#include "bsp_dwt.h"
#include <string.h>

extern volatile uint8_t cnt;

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

static void telem_seal_write_buf_ready(void)
{
    if (s_write_buf == NULL) {
        return;
    }
    s_write_buf->state = TELEM_BUF_READY;
    s_write_buf = NULL;
}

static void telem_write_small_frame(telem_buf_t *buf, uint16_t offset)
{
    float vals[4];
    uint8_t *p;

    vals[0] = (float)cnt;
    vals[1] = 0.0f;
    vals[2] = 0.0f;
    vals[3] = 4.0f;

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

    s_decim_cnt++;
    if (s_decim_cnt < TELEM_BRINGUP_DECIMATION) {
        return;
    }
    s_decim_cnt = 0u;

    if (!telem_acquire_write_buf()) {
        return;
    }

    buf = s_write_buf;

    if ((uint32_t)buf->used_bytes + TELEM_SMALL_FRAME_BYTES > TELEM_BUF_BYTES) {
        telem_seal_write_buf_ready();
        if (!telem_acquire_write_buf()) {
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
        memcpy(&buf->data[off + TELEM_CH1_BYTE_OFF], &ch1_raw, sizeof(ch1_raw));
        memcpy(&buf->data[off + TELEM_CH2_BYTE_OFF], &ch2_raw, sizeof(ch2_raw));
    }
    buf->used_bytes = (uint16_t)(buf->used_bytes + TELEM_SMALL_FRAME_BYTES);

    if ((uint32_t)buf->used_bytes + TELEM_SMALL_FRAME_BYTES > TELEM_BUF_BYTES) {
        telem_seal_write_buf_ready();
    }
}

void telem_bringup_try_send(void)
{
    telem_buf_t *buf;

    if (telem_dma_busy()) {
        return;
    }

    buf = telem_find_buf(TELEM_BUF_READY);
    if (buf == NULL || buf->used_bytes == 0u) {
        return;
    }

    if (HAL_UART_Transmit_DMA(&hlpuart1, buf->data, buf->used_bytes) != HAL_OK) {
        return;
    }

    buf->state = TELEM_BUF_SENDING;
    s_sending_buf = buf;
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
    time_cnt = 0u;
#if TELEM_BRINGUP_INCLUDE_SEQ
    s_seq = 0u;
#endif
}

void UART_DMA_DEBUG_TASK(void *argument)
{
    (void)argument;

    for (;;) {
        telem_bringup_try_send();
        osDelay(1);
    }
}
