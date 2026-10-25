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
#if M1_ID_LOCK_CAL_SWEEP
#include "deadband_id_cal.h"
#endif
#if M1_IDENT_ENABLE
#include "ident_module.h"
#endif
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_module.h"
#endif
#include "telem_ident_dump.h"
#include "telem_lut_dump.h"
#include "telem_table.h"
#include "bsp_axes.h"
#include "motor_math.h"
#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   0
#endif
#if M1_HFI_ENABLE
#include "observer/hfi_sqwave.h"
#endif
#include <string.h>

#ifndef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K           12u
#endif
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
#ifndef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION  2u
#endif
#ifndef TELEM_BRINGUP_DECIMATION
#define TELEM_BRINGUP_DECIMATION     M1_TELEM_BRINGUP_DECIMATION
#endif

#if TELEM_FRAME_LAYOUT_ID
#define TELEM_LAYOUT_ID_BYTES     4u
#else
#define TELEM_LAYOUT_ID_BYTES     0u
#endif

#if TELEM_BRINGUP_INCLUDE_SEQ
#define TELEM_SMALL_FRAME_BYTES   (4u + TELEM_LAYOUT_ID_BYTES + TELEM_BRINGUP_K * 4u + 4u)
#else
#define TELEM_SMALL_FRAME_BYTES   (TELEM_LAYOUT_ID_BYTES + TELEM_BRINGUP_K * 4u + 4u)
#endif

#if TELEM_BRINGUP_INCLUDE_SEQ
#define TELEM_CH1_BYTE_OFF        (8u + TELEM_LAYOUT_ID_BYTES)
#define TELEM_CH2_BYTE_OFF        (12u + TELEM_LAYOUT_ID_BYTES)
#else
#define TELEM_CH1_BYTE_OFF        (4u + TELEM_LAYOUT_ID_BYTES)
#define TELEM_CH2_BYTE_OFF        (8u + TELEM_LAYOUT_ID_BYTES)
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

#if TELEM_BRINGUP_INCLUDE_SEQ
    {
        uint32_t seq = s_seq++;
        memcpy(p, &seq, sizeof(seq));
        p += sizeof(seq);
    }
#endif

#if TELEM_FRAME_LAYOUT_ID
    {
        float layout_id = (float)telem_get_layout();

        memcpy(p, &layout_id, sizeof(layout_id));
        p += sizeof(layout_id);
    }
#endif

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

#if M1_VOFA_SIGNOFF_CH
    /*
     * 位置/MIT 签收档 12ch（LEGACY 回落；默认已走 TELEM_LAYOUT_SIGNOFF 表）。
     * T-2：与表驱动同源（ctx / motor_current），不再读已删的 dbg 镜像。
     */
    {
        const motor_context_t *c = NULL;
        bsp_axis_t *ax2 = bsp_axis(BSP_AXIS_M1);

        if ((ax2 != NULL) && (ax2->motor_ctx != NULL)) {
            c = (const motor_context_t *)ax2->motor_ctx;
        }
        vals[0] = (c != NULL) ? c->ia : 0.0f;
        vals[1] = (c != NULL) ? c->ib : 0.0f;
        vals[2] = (c != NULL) ? c->ic : 0.0f;
        vals[3] = (c != NULL) ? c->iq : 0.0f;
        vals[4] = motor_current_get_theta_fb_rad();
        vals[5] = motor_current_get_theta_mech_rad();
        vals[6] = (c != NULL) ? c->theta_ref_rad : 0.0f;
        vals[7] = vals[6] - vals[4];
        vals[8] = motor_current_get_enc_pll_omega_mech_rpm();
        vals[9] = (c != NULL) ? c->omega_ref : 0.0f;
        vals[10] = (c != NULL) ? c->iq_ref : 0.0f;
        vals[11] = (float)dbg.open_seq_phase;
    }
    return;
#endif
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
#if M1_VOFA_IDENT_DUTY_12CH && !M1_SPEED_LOOP_ENABLE
    vals[6] = dbg.foc_vd_est;
    vals[7] = dbg.foc_vq_est;
#if M1_LD_LQ_IDENT_ENABLE
    if (deadband_id_cal_in_ld_lq_ident()) {
        vals[8] = dbg.ld_lq_proc_grid;
        vals[9] = dbg.ld_lq_proc_code;
        vals[10] = dbg.ld_lq_L_est_uH;
    } else
#endif
    {
        vals[8] = dbg.foc_duty_ta;
        vals[9] = dbg.foc_duty_tb;
        vals[10] = dbg.foc_duty_tc;
    }
    vals[11] = (float)dbg.open_seq_phase;
#else
    vals[6] = dbg.foc_ud_out;
    vals[7] = dbg.foc_uq_out;
#if M1_SPEED_IDENT_ENABLE
    vals[8] = dbg.pll_omega_mech_rpm;
    vals[9] = dbg.outer_omega_ref;
#if M1_SPEED_IDENT_BODE_ENABLE
    if (dbg.open_seq_phase == 230u) {
        vals[10] = speed_ident_module_bode_freq_hz();
    } else {
        vals[10] = dbg.outer_iq_ref;
    }
#else
    vals[10] = dbg.outer_iq_ref;
#endif
#if M1_VOFA_CH11_ENC_RAW
    vals[11] = dbg.enc_raw;   /* AS5047 raw 0..16383；对。ch5=θ_el */
#else
    vals[11] = (float)dbg.open_seq_phase;
#endif
#elif M1_POS_MIT_COMBO_ENABLE && M1_VOFA_MIT_CH8_11 && M1_PLL_ENABLE
    vals[8] = dbg.pll_omega_mech_rpm;
    vals[9] = dbg.outer_theta_err_rad;
    vals[10] = dbg.outer_iq_ref;
    vals[11] = (float)dbg.open_seq_phase;
#elif M1_POS_MIT_COMBO_ENABLE && M1_PLL_ENABLE
    vals[8] = dbg.outer_theta_err_rad;
    vals[9] = dbg.outer_theta_mech_rad;
    if (dbg.outer_mode == (uint8_t)M1_OUTER_TORQUE) {
        vals[10] = dbg.outer_iq_ref;
    } else {
        vals[10] = dbg.outer_theta_ref_rad;
    }
    vals[11] = (float)dbg.open_seq_phase;
#elif M1_SPEED_LOOP_ENABLE && M1_VOFA_SPEED_CH8_11 && M1_PLL_ENABLE
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
#elif M1_VOFA_PLL_CH8_11 && M1_PLL_ENABLE
    vals[8] = dbg.pll_omega_mech_rpm;
    vals[9] = dbg.pll_omega_diff_rpm;
    vals[10] = dbg.pll_theta_err_rad;
    vals[11] = dbg.pll_omega_err_rpm;
#else
    vals[8] = dbg.foc_id_ref;
    vals[9] = dbg.foc_iq_ref;
#if M1_LD_LQ_IDENT_ENABLE
    if (deadband_id_cal_in_ld_lq_ident()) {
        /* VASI 过程 telem：ch8=grid ch9=proc_code ch10=运行 L_uH ch11=open_seq */
        vals[8] = dbg.ld_lq_proc_grid;
        vals[9] = dbg.ld_lq_proc_code;
        vals[10] = dbg.ld_lq_L_est_uH;
        vals[11] = (float)dbg.open_seq_phase;
    } else
#endif
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
    if (deadband_id_cal_in_rs_ident() || deadband_id_cal_in_ld_lq_ident() ||
        deadband_id_cal_in_ld_lq_pre_decay()) {
        vals[10] = dbg.rs_l_ident_lut_round;
    } else
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    if (deadband_id_cal_in_ld_lq_sweep()) {
        vals[10] = (float)dbg.ld_lq_angle_leg;
    } else
#endif
    {
        vals[10] = dbg.foc_svpwm_duty_dev;
    }
#elif M1_LD_LQ_MULTI_ANGLE_ENABLE
    if (deadband_id_cal_in_ld_lq_sweep()) {
        vals[10] = (float)dbg.ld_lq_angle_leg;
    } else {
        vals[10] = dbg.foc_svpwm_duty_dev;
    }
#else
    vals[10] = dbg.foc_svpwm_duty_dev;
#endif
    vals[11] = (float)dbg.open_seq_phase;
#endif
#endif
}
#define TELEM_FOC_UNIFIED_12CH_ACTIVE  1
#else
#define TELEM_FOC_UNIFIED_12CH_ACTIVE  0
#endif

#if !TELEM_FOC_UNIFIED_12CH_ACTIVE
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
static void telem_fill_open_ladder_12ch(float vals[TELEM_BRINGUP_K])
{
    vals[0] = dbg.foc_ud_out;
    vals[1] = dbg.foc_uq_out;
    vals[2] = dbg.foc_vd_est;
    vals[3] = dbg.foc_vq_est;
    vals[4] = dbg.foc_id;
    vals[5] = dbg.foc_iq;
    vals[6] = dbg.foc_pwm_ccr1;
    vals[7] = dbg.foc_pwm_ccr2;
    vals[8] = dbg.foc_pwm_ccr3;
    vals[9] = dbg.foc_svpwm_uref;
    vals[10] = (float)dbg.foc_svpwm_sector;
    vals[11] = (float)dbg.open_seq_phase;
}
#endif

#if M1_ID_LOCK_CAL_SWEEP
static void telem_fill_id_cal_12ch(float vals[TELEM_BRINGUP_K])
{
    vals[0] = dbg.foc_ia;
    vals[1] = dbg.foc_ib;
    vals[2] = dbg.foc_ic;
    vals[3] = dbg.foc_id;
    vals[4] = dbg.foc_iq;
    vals[5] = dbg.foc_theta_el;
    vals[6] = dbg.foc_ud_out;
    vals[7] = dbg.foc_uq_out;
    vals[8] = dbg.foc_duty_ta;
    vals[9] = dbg.foc_duty_tb;
    vals[10] = dbg.foc_duty_tc;
    vals[11] = dbg.foc_svpwm_duty_dev;
}
#endif
#endif /* !TELEM_FOC_UNIFIED_12CH_ACTIVE */

static void telem_fill_small_frame_vals(float vals[TELEM_BRINGUP_K])
{
    uint32_t k;

    for (k = 0u; k < TELEM_BRINGUP_K; k++) {
        vals[k] = 0.0f;
    }

#ifndef M1_VOFA_FLUX_ID_6CH
#define M1_VOFA_FLUX_ID_6CH          0
#endif
#ifndef M1_VOFA_CH11_ENC_RAW
#define M1_VOFA_CH11_ENC_RAW         0
#endif
#ifndef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH         0
#endif
#ifndef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH         0
#endif
#ifndef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH         0
#endif
#ifndef M1_VOFA_OBS_SMO_RAW_12CH
#define M1_VOFA_OBS_SMO_RAW_12CH     0
#endif
#ifndef M1_VOFA_IF_12CH
#define M1_VOFA_IF_12CH              0
#endif
#ifndef M1_VOFA_HFI_12CH
#define M1_VOFA_HFI_12CH             0
#endif
#ifndef M1_VOFA_HFI_ISR_CH
#define M1_VOFA_HFI_ISR_CH           0
#endif

#if M1_VOFA_IDENT_DUMP_ENABLE
    if (telem_ident_dump_next(vals, TELEM_BRINGUP_K)) {
        /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
        return;
    }
#endif
#if M1_VOFA_LUT_DUMP_ENABLE
    if (telem_lut_dump_next(vals, TELEM_BRINGUP_K)) {
        /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
        return;
    }
#endif

#if M1_HFI_ENABLE && M1_VOFA_HFI_12CH && (TELEM_BRINGUP_K >= 12u)
{
    /*
     * GATE 141 12ch（Park 直录；改映射须重录金样）：
     * ch0 θ_park−θ_enc [rad]  ch1 θ_park  ch2 θ_enc  ch3 HFI−enc [rad]
     * ch4 θ_smo−θ_hfi [rad]  ch5 ω_enc  ch6 ω_fb(进速度环)  ch7 Iq*
     * ch8 Iq  ch9 ω_ref  ch10 stage（M1_VOFA_HFI_ISR_CH=1 时改 isr_delta）
     * ch11 pub ss
     */
    {
        const float th_park = dbg.foc_theta_el;
        const float th_enc = dbg.enc_theta_el;

        vals[0] = motor_wrap_pi(th_park - th_enc);
        vals[1] = th_park;
        vals[2] = th_enc;
        vals[3] = dbg.hfi_theta_err;
        vals[4] = dbg.obs_pll_theta_err;
        /* 编码器 PLL 真值（勿用未维护的 dbg 裸读；与 get_enc_pll 同源）。 */
        vals[5] = motor_current_get_enc_pll_omega_mech_rpm();
        /* 141 无 OBS_SOFT_SWITCH 时 dbg.outer_omega_mech_rpm 不更新；读本拍速度环 fb。 */
        vals[6] = motor_current_get_pll_omega_mech_rpm();
        vals[7] = dbg.foc_iq_ref;
        vals[8] = dbg.foc_iq;
        vals[9] = dbg.outer_omega_ref;
#if M1_VOFA_HFI_ISR_CH
        vals[10] = (float)g_telem_dbg.isr_delta;
#else
        vals[10] = dbg.hfi_stage;
#endif
        vals[11] = dbg.obs_ss_spd_on;
    }
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
}
#endif

#if M1_VOFA_IF_12CH && (TELEM_BRINGUP_K >= 12u)
    /*
     * I/F→OBS 先角后速联调：
     * ch0 ω_enc  ch1 ω_ref  ch2 Iq_ref  ch3 θ_err(pll−enc监督)
     * ch4 ω_obs  ch5 speed_fb(进PI)  ch6 Iq  ch7 emag
     * ch8 Uq     ch9 θ_park  ch10 spd_on(0/1)  ch11 ss_state+0.1α
     * 判读：进 OBS 。ch11。.x 。ch10=0 。角切速未切；ch10。 。ch5贴ch4 。速已切。
     */
    vals[0] = dbg.pll_omega_mech_rpm;
    vals[1] = dbg.if_omega_cmd_rpm;
    /* I/F 释放。ch1 跟外环；|ω| 判，兼容反向（旧：outer>1 会把 。000 漏成 0。*/
    if ((dbg.if_omega_cmd_rpm > -1.0f) && (dbg.if_omega_cmd_rpm < 1.0f) &&
        ((dbg.outer_omega_ref > 1.0f) || (dbg.outer_omega_ref < -1.0f))) {
        vals[1] = dbg.outer_omega_ref;
    }
    vals[2] = dbg.foc_iq_ref;
#if M1_EMF_PLL_ENABLE
    vals[3] = dbg.obs_pll_theta_err;
#else
    vals[3] = dbg.if_theta_err_rad;
#endif
    vals[4] = dbg.obs_spd_pll_rpm;
    vals[5] = dbg.outer_omega_mech_rpm;
    vals[6] = dbg.foc_iq;
#if M1_EMF_PLL_ENABLE
    vals[7] = dbg.obs_smo_emag;
#else
    vals[7] = dbg.obs_pll_pd;
#endif
    vals[8] = dbg.foc_uq_out;
    vals[9] = dbg.foc_theta_el;
    vals[10] = dbg.obs_ss_spd_on;
#if M1_OBS_SOFT_SWITCH_ENABLE
    vals[11] = dbg.obs_ss_state + 0.1f * dbg.obs_ss_alpha;
#else
    vals[11] = (float)dbg.open_seq_phase;
#endif
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
#endif

#if M1_VOFA_OBS_SMO_RAW_12CH && (TELEM_BRINGUP_K >= 12u)
    /* 离线重放原料：i/u + θ/ω；附带在。e/PLL 便于核对 */
    vals[0] = dbg.obs_i_alpha;
    vals[1] = dbg.obs_i_beta;
    vals[2] = dbg.obs_u_alpha;
    vals[3] = dbg.obs_u_beta;
    vals[4] = dbg.foc_theta_el;
    vals[5] = dbg.pll_omega_mech_rpm;
    vals[6] = dbg.obs_smo_e_alpha;
    vals[7] = dbg.obs_smo_e_beta;
    vals[8] = dbg.obs_pll_theta_err;
    vals[9] = dbg.obs_smo_emag;
    vals[10] = dbg.obs_pll_theta_hat;
    vals[11] = dbg.obs_smo_lpf_hz;
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
#endif

#if M1_VOFA_OBS_PLL_12CH && (TELEM_BRINGUP_K >= 12u)
#if M1_EMF_PLL_USE_SMO
#if M1_OBS_SPD_PLL_ENABLE
    /* 高速软切验收：速度反馈 / 指令 / 角误。/ 状态（不再。PLL 对照。*/
    vals[0] = dbg.outer_omega_mech_rpm; /* 实际进速度环的反馈 */
    vals[1] = dbg.outer_omega_ref;
    vals[2] = dbg.foc_iq_ref;
    vals[3] = dbg.obs_pll_theta_err;
    vals[4] = dbg.foc_theta_el;
    vals[5] = dbg.obs_spd_pll_rpm;     /* θ̂→PLL。 kHz。*/
    vals[6] = dbg.obs_smo_emag;
    vals[7] = dbg.obs_pll_theta_hat;
    vals[8] = (float)g_telem_dbg.isr_delta;
    vals[9] = (float)g_telem_dbg.foc_delta;
    vals[10] = (float)g_telem_dbg.obs_delta;
#if M1_OBS_SOFT_SWITCH_ENABLE
    vals[11] = dbg.obs_ss_state + 0.1f * dbg.obs_ss_alpha;
#else
    vals[11] = dbg.obs_smo_lpf_hz;
#endif
#else
    /* SMO e 。PLL：中间量 eαβ + atan/PLL 对照 + cycle */
    vals[0] = dbg.obs_smo_e_alpha;
    vals[1] = dbg.obs_smo_e_beta;
    vals[2] = dbg.obs_smo_theta_err;
    vals[3] = dbg.obs_pll_theta_err;
    vals[4] = dbg.foc_theta_el;
    vals[5] = dbg.obs_pll_omega_el;
    vals[6] = dbg.obs_smo_emag;
    vals[7] = dbg.obs_pll_theta_hat;
    vals[8] = (float)g_telem_dbg.isr_delta;
    vals[9] = (float)g_telem_dbg.foc_delta;
    vals[10] = (float)g_telem_dbg.obs_delta;
#if M1_OBS_SOFT_SWITCH_ENABLE
    vals[11] = dbg.obs_ss_state + 0.1f * dbg.obs_ss_alpha;
#else
    vals[11] = dbg.obs_smo_lpf_hz;
#endif
#endif
#else
    /* Veq atan vs EMF-PLL + cycle */
    vals[0] = dbg.obs_i_alpha;
    vals[1] = dbg.obs_i_beta;
    vals[2] = dbg.obs_theta_err;
    vals[3] = dbg.obs_pll_theta_err;
    vals[4] = dbg.foc_theta_el;
    vals[5] = dbg.obs_pll_omega_el;
    vals[6] = dbg.obs_emag;
    vals[7] = dbg.obs_pll_theta_hat;
    vals[8] = (float)g_telem_dbg.isr_delta;
    vals[9] = (float)g_telem_dbg.foc_delta;
    vals[10] = (float)g_telem_dbg.obs_delta;
    vals[11] = (float)g_telem_dbg.enc_dma_cpu_delta;
#endif
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
#endif

#if M1_VOFA_OBS_SMO_12CH && (TELEM_BRINGUP_K >= 12u)
    /* 保留验收。+ DWT cycle（float 显示；isr 为上一拍完。tick。*/
    vals[0] = dbg.obs_i_alpha;
    vals[1] = dbg.obs_i_beta;
    vals[2] = dbg.obs_theta_err;       /* Veq err */
    vals[3] = dbg.obs_smo_theta_err;   /* SMO err 。主验。*/
    vals[4] = dbg.foc_theta_el;
    vals[5] = dbg.obs_omega_el;
    vals[6] = dbg.obs_smo_emag;
    vals[7] = dbg.obs_emag;           /* Veq |e| */
    vals[8] = (float)g_telem_dbg.isr_delta;
    vals[9] = (float)g_telem_dbg.foc_delta;
    vals[10] = (float)g_telem_dbg.obs_delta;
    vals[11] = (float)g_telem_dbg.enc_dma_cpu_delta;
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
#endif

#if M1_VOFA_OBS_VEQ_12CH && (TELEM_BRINGUP_K >= 12u)
    /* Veq 旁路：iαβ uαβ eαβ θ̂ θenc θerr |e| ωe ψinst */
    vals[0] = dbg.obs_i_alpha;
    vals[1] = dbg.obs_i_beta;
    vals[2] = dbg.obs_u_alpha;
    vals[3] = dbg.obs_u_beta;
    vals[4] = dbg.obs_e_alpha;
    vals[5] = dbg.obs_e_beta;
    vals[6] = dbg.obs_theta_hat;
    vals[7] = dbg.foc_theta_el;
    vals[8] = dbg.obs_theta_err;
    vals[9] = dbg.obs_emag;
    vals[10] = dbg.obs_omega_el;
    vals[11] = dbg.obs_psi_inst;
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
#endif

#if M1_VOFA_FLUX_ID_6CH && (TELEM_BRINGUP_K >= 6u)
    /* 有感稳速估 ψf：Id Iq Ud Uq ω_pll ω_ref @ D=5 。4 kHz */
    vals[0] = dbg.foc_id;
    vals[1] = dbg.foc_iq;
    vals[2] = dbg.foc_ud_out;
    vals[3] = dbg.foc_uq_out;
    vals[4] = dbg.pll_omega_mech_rpm;
    vals[5] = dbg.outer_omega_ref;
    /* P-1：打包移到任务侧 telem_bringup_pack_ready() */
    return;
#endif

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
#else
#if M1_VOFA_FOC_ABC
        vals[0] = dbg.foc_ia;
        vals[1] = dbg.foc_ib;
        vals[2] = dbg.foc_ic;
#else
        vals[0] = (float)dbg.adc_zeroed[0];
        vals[1] = (float)dbg.adc_zeroed[1];
        vals[2] = (float)dbg.adc_zeroed[2];
#endif
#if M1_IDENT_ENABLE
#if M1_IDENT_ID_CAL_BEFORE_STEP
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
        if (dbg.open_seq_phase >= M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE &&
            dbg.open_seq_phase <= M1_OPEN_PRE_ID_LADDER_DONE_PHASE) {
#if M1_TELEM_BRINGUP_K >= 12u
            telem_fill_open_ladder_12ch(vals);
#else
            vals[0] = dbg.foc_svpwm_uref;
            vals[1] = dbg.foc_svpwm_duty_dev;
            vals[2] = dbg.foc_svpwm_duty_ab;
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
            vals[3] = dbg.foc_id;
            vals[4] = dbg.foc_ud_out;
#else
            vals[3] = dbg.foc_iq;
            vals[4] = dbg.foc_uq_out;
#endif
            vals[5] = dbg.foc_theta_el;
#endif
        } else
#endif
        if (dbg.open_seq_phase >= 60u && dbg.open_seq_phase <= 84u) {
            vals[3] = dbg.foc_iq;
            vals[4] = ident_module_iq_ref_cmd();
            vals[5] = (dbg.open_seq_phase >= 62u && dbg.open_seq_phase < 73u) ?
                      ident_module_bode_freq_hz() :
                      dbg.foc_uq_pi;
        } else {
            vals[3] = dbg.foc_ud_pi;
            vals[4] = dbg.foc_id_ref;
            vals[5] = dbg.foc_theta_el;
        }
#else
#if M1_IDENT_BODE_AXIS_ID
        if (dbg.open_seq_phase >= 60u && dbg.open_seq_phase <= 79u) {
            vals[3] = dbg.foc_id;
            vals[4] = ident_module_id_ref_cmd();
            vals[5] = (dbg.open_seq_phase >= 62u && dbg.open_seq_phase < 73u) ?
                      ident_module_bode_freq_hz() :
                      dbg.foc_ud_pi;
        } else {
            vals[3] = dbg.foc_id;
            vals[4] = dbg.foc_id_ref;
            vals[5] = dbg.foc_theta_el;
        }
#else
        if (dbg.open_seq_phase >= 60u && dbg.open_seq_phase <= 79u) {
            vals[3] = dbg.foc_iq;
            vals[4] = ident_module_iq_ref_cmd();
            vals[5] = (dbg.open_seq_phase >= 62u && dbg.open_seq_phase < 73u) ?
                      ident_module_bode_freq_hz() :
                      dbg.foc_uq_pi;
        } else {
            vals[3] = dbg.foc_iq;
            vals[4] = ident_module_iq_ref_cmd();
            vals[5] = (dbg.open_seq_phase >= 62u && dbg.open_seq_phase < 73u) ?
                      ident_module_bode_freq_hz() :
                      dbg.foc_theta_el;
        }
#endif
#endif
#elif M1_ID_LOCK_CAL_SWEEP
#if M1_TELEM_BRINGUP_K >= 12u
        telem_fill_id_cal_12ch(vals);
#else
        vals[3] = dbg.foc_ud_pi;
        vals[4] = dbg.foc_id_ref;
        vals[5] = dbg.foc_theta_el;
#endif
#else
        vals[3] = dbg.foc_iq;
#if M1_VOFA_SECTOR_DIAG
        vals[4] = (float)svpwm_sector_from_uq_ud(dbg.foc_uq_out, 0.0f, dbg.foc_theta_el);
#else
        vals[4] = dbg.foc_id_ref;
#endif
        vals[5] = dbg.foc_theta_el;
#endif
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
#if TELEM_BRINGUP_INCLUDE_SEQ
    s_seq = 0u;
#endif
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
