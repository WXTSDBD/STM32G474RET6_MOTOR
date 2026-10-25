/**
 * @file app_uart_dma_debug.c
 * @date 2026-10-06
 * @brief 双缓冲组帧和 LPUART DMA 发送。

 *
 * 节拍限制见 app_uart_dma_debug.h 文件头。通道语义改了才重录金样。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "app_uart_dma_debug.h"
#include "cmsis_os.h"
#include "motor_context.h"
#include "motor_current.h"
#include "hal_bridge.h"
#include "time_port.h"
#include "dbg_monitor.h"
#include "encoder.h"
#include "foc_svpwm.h"
#include "encoder_spi_bus.h"
#include "phase_detect.h"
#include "bringup_bench.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"
#include "telem_ident_dump.h"
#include "telem_lut_dump.h"
#include "telem_table.h"
#include "bsp_axes.h"
#include "motor_math.h"
#if M1_HFI_ENABLE
#include "observer/hfi_sqwave.h"
#endif
#include <string.h>

#define TELEM_BRINGUP_K              M1_TELEM_BRINGUP_K
#define TELEM_BRINGUP_INCLUDE_SEQ    0u
#define TELEM_CPU_MHZ                160u

/** 单缓。4KB，双缓冲。8KB SRAM */
#define TELEM_BUF_BYTES              4096u

/**
 * JEOC 20kHz 下每 D 。tick 。1 个小帧（。motor_current_tick 同拍）。
 * D=2  。10kHz。2ch×52B 。520KB/s @ 6Mbps。
 * D=1  。20kHz；D=100 。200Hz（bringup 低压联调用）
 */
#ifndef TELEM_BRINGUP_DECIMATION
#define TELEM_BRINGUP_DECIMATION     M1_TELEM_BRINGUP_DECIMATION
#endif

#define TELEM_LAYOUT_ID_BYTES     0u

#define TELEM_SMALL_FRAME_BYTES   (TELEM_LAYOUT_ID_BYTES + TELEM_BRINGUP_K * 4u + 4u)

#define TELEM_CH1_BYTE_OFF        (4u + TELEM_LAYOUT_ID_BYTES)
#define TELEM_CH2_BYTE_OFF        (8u + TELEM_LAYOUT_ID_BYTES)

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


static uint32_t s_decim_cnt;

telem_dbg_t g_telem_dbg;

/* ---- P-1 遥测快照化（2026-10-18 22:05 / 22:20 改环形）：ISR 只写 12xfloat，打包/DMA 交给任务侧 ---- */
#ifndef TELEM_SNAP_DEPTH
#define TELEM_SNAP_DEPTH             32u   /* 32 帧 ≈ 3.2 ms @10 kHz，够任务侧 1 ms 周期排空 */
#endif

typedef struct {
    float vals[TELEM_BRINGUP_K];
} telem_snap_t;

static telem_snap_t s_snap[TELEM_SNAP_DEPTH];
static volatile uint16_t s_snap_w;    /* ISR 写指针 */
static volatile uint16_t s_snap_r;    /* 任务读指针 */
static uint32_t s_snap_drop;          /* 环满丢帧计数（调试用） */

/** @brief ISR 侧：把本拍 12 个采样值写进快照环（不打包、不碰 UART 缓冲）。 */
static void telem_snapshot_commit(const float vals[TELEM_BRINGUP_K])
{
    uint16_t w = s_snap_w;
    uint16_t nw = (uint16_t)((w + 1u) % TELEM_SNAP_DEPTH);

    memcpy(s_snap[w].vals, vals, sizeof(s_snap[w].vals));
    __DMB();
    if (nw == s_snap_r) {
        /* 环满：丢最旧一帧，计数记进 acquire_fail（P-1 起它同时表示遥测丢帧） */
        s_snap_r = (uint16_t)((s_snap_r + 1u) % TELEM_SNAP_DEPTH);
        s_snap_drop++;
        g_telem_dbg.acquire_fail++;
    }
    s_snap_w = nw;
}

/** 调试：最近一。ch2_wire（isr_delta。*/
volatile uint32_t time_cnt;

static uint32_t s_prof_kick_cyccnt;
static uint32_t s_prof_f1_delta;

static void telem_enc_profile_cb(const encoder_t *e, const enc_profile_event_t *ev)
{
    if (e == NULL || ev == NULL) {
        return;
    }

    g_telem_dbg.enc_spi_state = encoder_spi_bus_is_busy(e->bus);

    switch (ev->ev) {
    case ENC_EVT_KICK_DONE:
        g_telem_dbg.enc_dma_kick_delta = ev->aux;
        s_prof_kick_cyccnt = ev->cyccnt;
        g_telem_dbg.enc_kick_cnt++;
        g_telem_dbg.enc_dma_busy = 1U;
        g_telem_dbg.enc_dma_phase = 1U;
        break;
    case ENC_EVT_F1_DONE:
        s_prof_f1_delta = ev->aux;
        g_telem_dbg.enc_dma_f1_cb_delta = ev->aux;
        g_telem_dbg.enc_dma_phase = 2U;
        break;
    case ENC_EVT_F2_DONE:
        /* 双帧完成或 KTH 单帧完成 */
        g_telem_dbg.enc_dma_f2_cb_delta = ev->aux;
        g_telem_dbg.enc_dma_cpu_delta = s_prof_f1_delta + ev->aux;
        g_telem_dbg.enc_dma_seq_delta = ev->cyccnt - s_prof_kick_cyccnt;
        if (g_telem_dbg.enc_dma_seq_delta > g_telem_dbg.enc_dma_seq_delta_max) {
            g_telem_dbg.enc_dma_seq_delta_max = g_telem_dbg.enc_dma_seq_delta;
        }
        g_telem_dbg.enc_total_delta = g_telem_dbg.isr_delta + g_telem_dbg.enc_dma_cpu_delta;
        g_telem_dbg.enc_raw = encoder_get_raw(e);
        g_telem_dbg.enc_rx_word1 = g_telem_dbg.enc_raw;
        g_telem_dbg.enc_cplt_cnt++;
        g_telem_dbg.enc_dma_busy = 0U;
        g_telem_dbg.enc_dma_phase = 0U;
        break;
    case ENC_EVT_KICK_SKIP_BUSY:
        g_telem_dbg.enc_kick_skip_busy++;
        g_telem_dbg.enc_dma_busy = encoder_xfer_busy(e);
        g_telem_dbg.enc_dma_phase = g_telem_dbg.enc_dma_busy;
        break;
    case ENC_EVT_ERROR:
        g_telem_dbg.enc_err_cnt++;
        g_telem_dbg.enc_dma_busy = 0U;
        g_telem_dbg.enc_dma_phase = 0U;
        break;
    default:
        break;
    }
}

/**
 * @brief 绑编码器事件回调。
 */
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

static void telem_write_frame_vals(telem_buf_t *buf, uint16_t offset, const float vals[TELEM_BRINGUP_K])
{
    uint8_t *p = &buf->data[offset];



    memcpy(p, vals, sizeof(float) * TELEM_BRINGUP_K);
    p += sizeof(float) * TELEM_BRINGUP_K;
    memcpy(p, s_justfloat_tail, sizeof(s_justfloat_tail));
}

#if (M1_VOFA_UNIFIED_12CH != 0) && (M1_TELEM_BRINGUP_K >= 12u) && \
    (M1_ID_LOCK_CAL_SWEEP || M1_IDENT_ENABLE || M1_SPEED_LOOP_ENABLE || \
     M1_OPEN_UD_PRE_ID_CAL_ENABLE || M1_OPEN_UQ_PRE_ID_CAL_ENABLE)
/** bringup 统一 12 通道（Id cal / ident / 开环阶梯共用） */
static void telem_fill_foc_unified_12ch(float vals[TELEM_BRINGUP_K])
{
    {
        const motor_context_t *ctx = NULL;
        bsp_axis_t *ax = bsp_axis(BSP_AXIS_M1);

        if ((ax != NULL) && (ax->motor_ctx != NULL)) {
            ctx = (const motor_context_t *)ax->motor_ctx;
        }
        /* T-1：表驱动优先；LEGACY 才落下面旧 #elif */
        if (telem_table_fill(vals, ctx) != 0u) {
            return;
        }
    }

    {
        const motor_context_t *c = NULL;
        bsp_axis_t *ax2 = bsp_axis(BSP_AXIS_M1);

        if ((ax2 != NULL) && (ax2->motor_ctx != NULL)) {
            c = (const motor_context_t *)ax2->motor_ctx;
        }
        vals[0] = (c != NULL) ? c->ia : 0.0f;
        vals[1] = (c != NULL) ? c->ib : 0.0f;
        vals[2] = (c != NULL) ? c->ic : 0.0f;
        vals[3] = (c != NULL) ? c->id : 0.0f;
        vals[4] = (c != NULL) ? c->iq : 0.0f;
    }
    vals[5] = dbg.foc_theta_el;
    vals[6] = dbg.foc_ud_out;
    vals[7] = dbg.foc_uq_out;
    if (dbg.outer_mode == (uint8_t)M1_OUTER_POSITION ||
        dbg.outer_mode == (uint8_t)M1_OUTER_MIT) {
        vals[8] = dbg.outer_theta_err_rad;
        vals[9] = dbg.outer_theta_mech_rad;
        vals[10] = dbg.outer_theta_ref_rad;
        vals[11] = dbg.outer_omega_ref;
    } else {
        vals[8] = dbg.pll_omega_mech_rpm;
        vals[9] = dbg.outer_theta_mech_rad;
        vals[10] = dbg.outer_omega_ref;
        vals[11] = dbg.outer_omega_ref - dbg.pll_omega_mech_rpm;
    }
}
#define TELEM_FOC_UNIFIED_12CH_ACTIVE  1
#endif


static void telem_fill_small_frame_vals(float vals[TELEM_BRINGUP_K])
{
    uint32_t k;

    for (k = 0u; k < TELEM_BRINGUP_K; k++) {
        vals[k] = 0.0f;
    }










    if (g_phase_cal_active || g_cal_hold) {
        vals[0] = (float)dbg.adc_zeroed[0];
        vals[1] = (float)dbg.adc_zeroed[1];
        vals[2] = (float)dbg.adc_zeroed[2];
        vals[3] = (float)dbg.phase_cal_pwm_idx;
        vals[4] = (float)dbg.phase_cal_delta_idx;
        vals[5] = (float)dbg.phase_cal_st;
#if TELEM_FOC_UNIFIED_12CH_ACTIVE
        vals[6] = dbg.foc_ud_out;
        vals[7] = dbg.foc_uq_pi;
        vals[8] = dbg.foc_id_ref;
        vals[9] = dbg.foc_iq_ref;
        vals[10] = dbg.foc_svpwm_duty_dev;
        vals[11] = (float)dbg.open_seq_phase;
#endif
    } else {
#if TELEM_FOC_UNIFIED_12CH_ACTIVE
        telem_fill_foc_unified_12ch(vals);
#endif /* !TELEM_FOC_UNIFIED_12CH_ACTIVE */
    }

    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
}

/**
 * @brief 任务侧：把最近一帧 ISR 快照打包进写缓冲（P-1：打包搬出热路径）。
 * @note  只在任务上下文调用（UART_DMA_DEBUG_TASK 循环里），与 ISR 无共享写。
 */
static void telem_bringup_pack_ready(void)
{
    telem_buf_t *buf;
    uint16_t off;
    float vals[TELEM_BRINGUP_K];

    while (s_snap_r != s_snap_w) {
        uint16_t r = s_snap_r;

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

        memcpy(vals, s_snap[r].vals, sizeof(vals));
        __DMB();
        s_snap_r = (uint16_t)((r + 1u) % TELEM_SNAP_DEPTH);

        off = buf->used_bytes;
        telem_write_frame_vals(buf, off, vals);
        buf->used_bytes = (uint16_t)(buf->used_bytes + TELEM_SMALL_FRAME_BYTES);
        g_telem_dbg.ch1_wire = g_telem_dbg.cyccnt_end;
        g_telem_dbg.ch2_wire = g_telem_dbg.isr_delta;
        time_cnt = g_telem_dbg.isr_delta;

        if ((uint32_t)buf->used_bytes + TELEM_SMALL_FRAME_BYTES > TELEM_BUF_BYTES) {
            telem_seal_write_buf_ready();
        }
        g_telem_dbg.tick_frame_ok++;
    }
}

/**
 * @brief ISR 侧：只做分频 + 取 12 个值 + 写快照（P-1 后不再打包、不碰 UART 缓冲）。
 */
void telem_bringup_tick(void)
{
    float vals[TELEM_BRINGUP_K];

    g_telem_dbg.tick_total++;
    s_decim_cnt++;
    if (s_decim_cnt < TELEM_BRINGUP_DECIMATION) {
        g_telem_dbg.tick_decim_skip++;
        return;
    }
    s_decim_cnt = 0u;

    telem_fill_small_frame_vals(vals);
    telem_snapshot_commit(vals);
}

/**
 * @brief 任务侧若有就绪缓冲则启动 DMA。
 */
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

/**
 * @brief 初始化双缓冲和 UART DMA。
 */
void telem_bringup_init(void)
{
    uint32_t i;

    time_port_init(TELEM_CPU_MHZ);
    telem_table_init();

    for (i = 0u; i < 2u; i++) {
        s_bufs[i].used_bytes = 0u;
        s_bufs[i].state = TELEM_BUF_UNLOCKED;
    }

    s_write_buf = NULL;
    s_sending_buf = NULL;
    s_decim_cnt = 0u;
    s_snap_w = 0u;
    s_snap_r = 0u;
    s_snap_drop = 0u;
    g_telem_dbg.isr_t0 = 0u;
    g_telem_dbg.isr_t1 = 0u;
    g_telem_dbg.cyccnt_end = 0u;
    g_telem_dbg.isr_delta = 0u;
    g_telem_dbg.foc_delta = 0u;
    g_telem_dbg.obs_delta = 0u;
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
}

void UART_DMA_DEBUG_TASK(void *argument)
{
    (void)argument;

    for (;;) {
        g_telem_dbg.uart_task_loops++;
        telem_bringup_pack_ready();   /* P-1：打包在任务上下文，ISR 只写快照环 */
        telem_bringup_try_send();
        osDelay(1);
    }
}
