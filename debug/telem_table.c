/**
 * @file telem_table.c
 * @date 2026-10-07
 * @brief 遥测 layout 表与 getter（T-2：直读 ctx / motor_current 状态）。
 *
 * 签收 SIGNOFF 12ch：
 *   ia/ib/ic/iq ← ctx 实测
 *   theta_fb/mech ← motor_current getter
 *   theta_ref/err、omega_ref、iq_ref ← ctx
 *   omega_pll ← 编码器 PLL（非 s_speed_fb）
 *   ch11：默认 mark；M1_VOFA_SIGNOFF_ENC_RAW=1 时改为 enc_raw（编码器冒烟录波）
 *   M1_VOFA_KTH_SPEED_DIAG=1 时：ch7=iq_ref、ch10=isr_delta（速度对照）
 *
 * FOC_DQ / IDENT 仍有部分通道读 dbg（ud/uq/duty 等，本刀未迁）。
 */

#include "telem_table.h"

#include <stddef.h>

#include "app_uart_dma_debug.h"
#include "motor_params_m1.h"
#include "dbg_monitor.h"
#include "motor_current.h"
#if M1_EXP_FRAMEWORK_ENABLE
#include "experiment/exp_runner.h"
#include "experiment/exp_act_basic.h"
#endif
#include "motor_math.h"
#include "pwm_port.h"
#include "observer/observer_composite.h"
#include "app_axis_jeoc.h"

static uint8_t s_layout = TELEM_LAYOUT_LEGACY;

/* ---- getters：A 类直读状态；D 类 mark 仍读 dbg ---- */

static float tl_ia(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->ia : 0.0f;
}

static float tl_ib(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->ib : 0.0f;
}

static float tl_ic(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->ic : 0.0f;
}

static float tl_id(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->id : 0.0f;
}

static float tl_iq(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->iq : 0.0f;
}

static float tl_theta_el(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_theta_el;
}

static float tl_ud_out(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_ud_out;
}

static float tl_uq_out(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_uq_out;
}

static float tl_id_ref(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->id_ref : 0.0f;
}

static float tl_iq_ref_foc(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->iq_ref : 0.0f;
}

static float tl_duty_dev(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_svpwm_duty_dev;
}

static float tl_theta_fb(const motor_context_t *ctx)
{
    (void)ctx;
    return motor_current_get_theta_fb_rad();
}

static float tl_theta_mech(const motor_context_t *ctx)
{
    (void)ctx;
    return motor_current_get_theta_mech_rad();
}

static float tl_theta_ref(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->theta_ref_rad : 0.0f;
}

static float tl_theta_err(const motor_context_t *ctx)
{
    if (ctx == NULL) {
        return 0.0f;
    }
    return ctx->theta_ref_rad - motor_current_get_theta_fb_rad();
}

static float tl_omega_pll(const motor_context_t *ctx)
{
    (void)ctx;
    return motor_current_get_enc_pll_omega_mech_rpm();
}

static float tl_omega_ref(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->omega_ref : 0.0f;
}

static float tl_iq_ref_outer(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->iq_ref : 0.0f;
}

static float tl_mark(const motor_context_t *ctx)
{
    (void)ctx;
    return (float)dbg.open_seq_phase;
}

/**
 * @brief 编码器 raw（KTH 0…65535，AS5047 0…16383）。
 */
static float tl_enc_raw(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.enc_raw;
}

/**
 * @brief 锁转子标定得到的电角 add，单位 rad。
 */
static float tl_enc_cal_add(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.enc_cal_add;
}

/**
 * @brief FOC ISR 周期差，单位 DWT cycle。
 */
static float tl_isr_delta(const motor_context_t *ctx)
{
    (void)ctx;
    return (float)g_telem_dbg.isr_delta;
}

/**
 * @brief PLL ω 与差分 ω 之差，单位 rpm（速度观测质量）。
 */
static float tl_pll_omega_err(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.pll_omega_err_rpm;
}

static float tl_vd_est(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_vd_est;
}

static float tl_vq_est(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_vq_est;
}

static float tl_duty_ta(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_duty_ta;
}

static float tl_duty_tb(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_duty_tb;
}

static float tl_duty_tc(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_duty_tc;
}

/* ---- OPT12（2026-10-18 §7 定稿）：优化期专用 12ch ---- */

/** @brief theta_park - theta_enc（wrap），单位 rad。 */
static float tl_opt_theta_err(const motor_context_t *ctx)
{
    (void)ctx;
    return motor_wrap_pi(dbg.foc_theta_el - dbg.enc_theta_el);
}

/** @brief ISR 全程 DWT cycle。 */
static float tl_opt_isr_delta(const motor_context_t *ctx)
{
    (void)ctx;
    return (float)g_telem_dbg.isr_delta;
}

/** @brief FOC 计算段 DWT cycle（Clark->PI->SVPWM/kick，不含观测器）。 */
static float tl_opt_foc_delta(const motor_context_t *ctx)
{
    (void)ctx;
    return (float)g_telem_dbg.foc_delta;
}

/** @brief d 轴电流实测值（诊断有感/无感电流环差异用；替换原 ch9 cnt_dir）。 */
static float tl_opt_id_dbg(const motor_context_t *ctx)
{
    return (ctx != NULL) ? ctx->id : 0.0f;
}

/**
 * @brief PI 的 d 轴输出（进桥之前；替换原 ch8 foc_delta）。
 * 与 ch5 ud_out 对比：已实测两段都 =1.0000，说明 PI 之后无叠加/缩放。
 */
static float tl_opt_ud_pi_dbg(const motor_context_t *ctx)
{
    (void)ctx;
    return dbg.foc_ud_pi;
}

/** @brief [台架诊断] FOC ISR 计数（ch9，原 id）。 */
static float tl_opt_foc_cnt_dbg(const motor_context_t *ctx)
{
    (void)ctx;
    return (float)(motor_current_get_foc_cnt() % 30000u);
}

/**
 * @brief [diag] packed experiment state: 100000*outer_run + 10000*armed
 *        + 1000*active_id + 10*theta_active + (mode==CURRENT_LOOP).
 */
static float tl_opt_diag_wref(const motor_context_t *ctx)
{
    float v = 0.0f;
    v += 100000.0f * (float)motor_current_outer_run_enable();
    v += 10000.0f * (float)exp_runner_is_armed();
    v += 1000.0f * (float)exp_runner_active_id();
    v += 10.0f * (float)exp_act_theta_is_active();
    v += ((ctx != NULL) && (ctx->mode == M1_CTRL_CURRENT_LOOP)) ? 1.0f : 0.0f;
    return v;
}

/** @brief status 位：bit0 hfi_stage>0, bit1 有感反馈, bit2 JQOVF, bit3 漏拍。 */
static float tl_opt_status(const motor_context_t *ctx)
{
    uint32_t b = 0u;

    (void)ctx;
    if (dbg.hfi_stage > 0.0f) {
        b |= 0x01u;
    }
    if (observer_sensed_fb_active() != 0u) {
        b |= 0x02u;
    }
    b |= (app_axis_jeoc_diag_bits() & 0x0Cu);
    return (float)b;
}

/** @brief 真正进速度环的反馈，单位 rpm（无感=观测速）。 */
static float tl_opt_omega_fb(const motor_context_t *ctx)
{
    (void)ctx;
    return motor_current_get_pll_omega_mech_rpm();
}

#if M1_VOFA_SIGNOFF_ENC_RAW
/** 编码器冒烟/标定：… ω_pll ω_ref enc_cal_add enc_raw */
static const telem_ch_t s_layout_signoff[TELEM_TABLE_CH_N] = {
    { "ia",          tl_ia },
    { "ib",          tl_ib },
    { "ic",          tl_ic },
    { "iq",          tl_iq },
    { "theta_fb",    tl_theta_fb },
    { "theta_mech",  tl_theta_mech },
    { "theta_el",    tl_theta_el },
    { "theta_err",   tl_theta_err },
    { "omega_pll",   tl_omega_pll },
    { "omega_ref",   tl_omega_ref },
    { "enc_cal_add", tl_enc_cal_add },
    { "enc_raw",     tl_enc_raw },
};
#else
/** 签收：ia ib ic iq θ_fb θ_mech θ_ref θ_err ω_pll ω_ref iq_ref mark */
static const telem_ch_t s_layout_signoff[TELEM_TABLE_CH_N] = {
    { "ia",         tl_ia },
    { "ib",         tl_ib },
    { "ic",         tl_ic },
    { "iq",         tl_iq },
    { "theta_fb",   tl_theta_fb },
    { "theta_mech", tl_theta_mech },
    { "theta_ref",  tl_theta_ref },
    { "theta_err",  tl_theta_err },
    { "omega_pll",  tl_omega_pll },
    { "omega_ref",  tl_omega_ref },
    { "iq_ref",     tl_iq_ref_outer },
    { "mark",       tl_mark },
};
#endif

/** 旧统一前半 + id/iq_ref/duty：ia ib ic id iq θ ud uq id_ref iq_ref duty_dev mark */
static const telem_ch_t s_layout_foc_dq[TELEM_TABLE_CH_N] = {
    { "ia",       tl_ia },
    { "ib",       tl_ib },
    { "ic",       tl_ic },
    { "id",       tl_id },
    { "iq",       tl_iq },
    { "theta_el", tl_theta_el },
    { "ud_out",   tl_ud_out },
    { "uq_out",   tl_uq_out },
    { "id_ref",   tl_id_ref },
    { "iq_ref",   tl_iq_ref_foc },
    { "duty_dev", tl_duty_dev },
    { "mark",     tl_mark },
};

/**
 * 速度环：ia ib ic id iq θ ud uq ω ω* ch10 mark。
 * M1_VOFA_SPEED_ISR_CH=1 时 ch10=isr_delta（DWT cycle），否则 iq_ref。
 */
static const telem_ch_t s_layout_speed[TELEM_TABLE_CH_N] = {
    { "ia",        tl_ia },
    { "ib",        tl_ib },
    { "ic",        tl_ic },
    { "id",        tl_id },
    { "iq",        tl_iq },
    { "theta_el",  tl_theta_el },
    { "ud_out",    tl_ud_out },
    { "uq_out",    tl_uq_out },
    { "omega_pll", tl_omega_pll },
    { "omega_ref", tl_omega_ref },
#if M1_VOFA_SPEED_ISR_CH
    { "isr_delta", tl_isr_delta },
#else
    { "iq_ref",    tl_iq_ref_outer },
#endif
    { "mark",      tl_mark },
};

/** 辨识 duty：ia ib ic id iq θ vd vq ta tb tc mark */
/**
 * OPT12：theta_err / enc_raw / omega_ref / iq / iq_ref / ud / uq /
 * isr_delta / foc_delta / cnt|DIR / status / omega_fb
 * ch1 原 omega_enc（编码器 PLL）与有感段 omega_fb 重复；改 enc_raw 供离线差分转速。
 */
static const telem_ch_t s_layout_opt[TELEM_TABLE_CH_N] = {
    { "theta_park_err", tl_opt_theta_err },
    { "enc_raw",        tl_enc_raw },
    { "omega_ref",      tl_omega_ref },
    { "iq",             tl_iq },
    { "iq_ref",         tl_iq_ref_foc },
    { "ud_out",         tl_ud_out },
    { "uq_out",         tl_uq_out },
    { "isr_delta",      tl_opt_isr_delta },
    { "ud_pi",          tl_opt_ud_pi_dbg },   /* PI 输出 vs ch5 ud_out */
    { "id",             tl_opt_id_dbg },      /* d 轴电流 */
    { "status",         tl_opt_status },
    { "omega_fb",       tl_opt_omega_fb },
};

static const telem_ch_t s_layout_ident_duty[TELEM_TABLE_CH_N] = {
    { "ia",       tl_ia },
    { "ib",       tl_ib },
    { "ic",       tl_ic },
    { "id",       tl_id },
    { "iq",       tl_iq },
    { "theta_el", tl_theta_el },
    { "vd_est",   tl_vd_est },
    { "vq_est",   tl_vq_est },
    { "duty_ta",  tl_duty_ta },
    { "duty_tb",  tl_duty_tb },
    { "duty_tc",  tl_duty_tc },
    { "mark",     tl_mark },
};

typedef struct {
    uint8_t id;
    const telem_ch_t *ch;
} telem_layout_slot_t;

static const telem_layout_slot_t s_layouts[] = {
    { TELEM_LAYOUT_FOC_DQ,     s_layout_foc_dq },
    { TELEM_LAYOUT_SIGNOFF,    s_layout_signoff },
    { TELEM_LAYOUT_SPEED,      s_layout_speed },
    { TELEM_LAYOUT_IDENT_DUTY, s_layout_ident_duty },
    { TELEM_LAYOUT_OPT,        s_layout_opt },
};

static const telem_ch_t *telem_table_find(uint8_t id)
{
    uint32_t i;

    for (i = 0u; i < (sizeof(s_layouts) / sizeof(s_layouts[0])); i++) {
        if (s_layouts[i].id == id) {
            return s_layouts[i].ch;
        }
    }
    return NULL;
}

static uint8_t telem_table_default_layout(void)
{
#if M1_VOFA_OPT12
    return TELEM_LAYOUT_OPT;
#elif M1_VOFA_SIGNOFF_CH
    return TELEM_LAYOUT_SIGNOFF;
#elif M1_VOFA_IDENT_DUTY_12CH && !M1_SPEED_LOOP_ENABLE
    return TELEM_LAYOUT_IDENT_DUTY;
#elif M1_SPEED_LOOP_ENABLE && M1_VOFA_SPEED_CH8_11 && M1_PLL_ENABLE
    return TELEM_LAYOUT_SPEED;
#elif M1_SPEED_LOOP_ENABLE && M1_PLL_ENABLE && !M1_POS_MIT_COMBO_ENABLE
    return TELEM_LAYOUT_SPEED;
#else
    return TELEM_LAYOUT_FOC_DQ;
#endif
}

void telem_table_init(void)
{
    s_layout = telem_table_default_layout();
    if (telem_table_find(s_layout) == NULL) {
        s_layout = TELEM_LAYOUT_FOC_DQ;
    }
}

void telem_set_layout(uint8_t layout_id)
{
    if ((layout_id != TELEM_LAYOUT_LEGACY) &&
        (telem_table_find(layout_id) == NULL)) {
        s_layout = telem_table_default_layout();
        return;
    }
    s_layout = layout_id;
}

uint8_t telem_get_layout(void)
{
    return s_layout;
}

uint8_t telem_table_fill(float vals[TELEM_TABLE_CH_N],
                         const motor_context_t *ctx)
{
    const telem_ch_t *ch;
    uint32_t i;

    if (vals == NULL) {
        return 0u;
    }
    if (s_layout == TELEM_LAYOUT_LEGACY) {
        return 0u;
    }

    ch = telem_table_find(s_layout);
    if (ch == NULL) {
        s_layout = TELEM_LAYOUT_FOC_DQ;
        ch = s_layout_foc_dq;
    }

    for (i = 0u; i < TELEM_TABLE_CH_N; i++) {
        vals[i] = ch[i].get(ctx);
    }
    return 1u;
}
