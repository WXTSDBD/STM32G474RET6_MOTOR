/**
 * @file app_uart_dma_debug.c
 * @brief UART_DMA_DEBUG 任务：双缓冲 + DMA 发送（Step 2b + Step 4）
 *
 * 发送路径（仅 RTOS 任务，不在 ISR 里启动 DMA）：
 *   telem_bringup_tick()  — 写入小帧到双缓冲（当前在任务里调用，后期可改 TIM1 ISR）
 *   telem_bringup_try_send() — READY 且 UART 空闲时 HAL_UART_Transmit_DMA
 *   HAL_UART_TxCpltCallback — SENDING → UNLOCKED
 *
 * 测试通道（不接电机）：
 *   ch0=(float)cnt  ch1=斜坡  ch2=2.0  ch3=4.0
 *
 * VOFA+：6000000，JustFloat，CH_COUNT=4（INCLUDE_SEQ=0）或 5（=1 且隐藏 ch0）
 */

#include "cmsis_os.h"
#include "usart.h"
#include <string.h>

extern volatile uint8_t cnt;

#define TELEM_BRINGUP_K           4u
#define TELEM_BRINGUP_INCLUDE_SEQ 0u
#define TELEM_BRINGUP_RAMP_STEP   0.1f

/** 单缓冲 4KB，双缓冲共 8KB SRAM */
#define TELEM_BUF_BYTES           4096u

/**
 * 写缓冲降采样：任务每 1ms 调一次 tick，D=100 → 约 100Hz 写入小帧
 * 填满满缓冲(20B/帧)约 2s；D=1000 → 约 10Hz 写入，约 20s 填满
 */
#define TELEM_BRINGUP_DECIMATION  100u

#if TELEM_BRINGUP_INCLUDE_SEQ
#define TELEM_SMALL_FRAME_BYTES   (4u + TELEM_BRINGUP_K * 4u + 4u)
#else
#define TELEM_SMALL_FRAME_BYTES   (TELEM_BRINGUP_K * 4u + 4u)
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

static float s_ramp_ch1;
static uint32_t s_decim_cnt;

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
    vals[1] = s_ramp_ch1;
    s_ramp_ch1 += TELEM_BRINGUP_RAMP_STEP;
    vals[2] = 2.0f;
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

    telem_write_small_frame(buf, buf->used_bytes);
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

static void telem_bringup_init(void)
{
    uint32_t i;

    for (i = 0u; i < 2u; i++) {
        s_bufs[i].used_bytes = 0u;
        s_bufs[i].state = TELEM_BUF_UNLOCKED;
    }

    s_write_buf = NULL;
    s_sending_buf = NULL;
    s_ramp_ch1 = 0.0f;
    s_decim_cnt = 0u;
#if TELEM_BRINGUP_INCLUDE_SEQ
    s_seq = 0u;
#endif
}

void UART_DMA_DEBUG_TASK(void *argument)
{
    (void)argument;

    telem_bringup_init();

    for (;;) {
        telem_bringup_tick();
        telem_bringup_try_send();
        osDelay(1);
    }
}
