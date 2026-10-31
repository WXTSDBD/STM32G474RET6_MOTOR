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
#include "l_hfi_ident.h"
#include "telemetry/obs_preset.h"

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

/**
 * @brief 去掉双轮 +100 后的 open_seq 基码。
 */
static uint8_t tl_open_seq_base(void)
{
    const uint8_t m = dbg.open_seq_phase;

    return (m >= 100u) ? (uint8_t)(m - 100u) : m;
}

/**
 * @brief L 段基码：55 本轮 shout / 56 轮间 / 57 跑中 / 58 OK / 59 FAIL / 163。
 */
static uint8_t tl_ident_l_overlay(void)
{
    const uint8_t m = tl_open_seq_base();

    return ((m == 55u) || (m == 56u) || (m == 57u) || (m == 58u) ||
            (m == 59u) || (m == 163u))
               ? 1u
               : 0u;
}

/**
 * @brief 已出 L 且非注入中：粘住 Ld/Lq 供双轮对照（含轮间 Rs）。
 */
static uint8_t tl_ident_l_sticky(void)
{
    const uint8_t m = tl_open_seq_base();

    if (dbg.ld_lq_ok == 0u) {
        return 0u;
    }
    if (m == 57u) {
        return 0u;
    }
    if ((m >= 1u) && (m <= 30u)) {
        return 0u;
    }
    return 1u;
}

/**
 * @brief IDENT ch6：L 段看 Ud 阶跃；其余仍 Vd_est。
 */
static float tl_ident_ch6(const motor_context_t *ctx)
{
    (void)ctx;
    if (tl_ident_l_overlay() != 0u) {
        return dbg.foc_ud_out;
    }
    return dbg.foc_vd_est;
}

/**
 * @brief IDENT ch7：L 段看 Uq 阶跃；其余仍 Vq_est。
 */
static float tl_ident_ch7(const motor_context_t *ctx)
{
    (void)ctx;
    if (tl_ident_l_overlay() != 0u) {
        return dbg.foc_uq_out;
    }
    return dbg.foc_vq_est;
}

/**
 * @brief IDENT ch8：Ld_uH / grid；Pass0=id_ref。
 */
static float tl_ident_ch8(const motor_context_t *ctx)
{
    const uint8_t m = tl_open_seq_base();

    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_ld_uH();
    }
    if (tl_ident_l_sticky() != 0u) {
        return dbg.ld_lq_proc_last_l_uH;
    }
    if (tl_ident_l_overlay() != 0u) {
        if ((m == 58u) || (m == 59u)) {
            return dbg.ld_lq_proc_last_l_uH;
        }
        return dbg.ld_lq_proc_grid;
    }
    if ((m >= 1u) && (m <= 30u)) {
        return (ctx != NULL) ? ctx->id_ref : dbg.foc_id_ref;
    }
    return dbg.foc_duty_ta;
}

/**
 * @brief IDENT ch9：Lq_uH / proc；Pass0=id。
 */
static float tl_ident_ch9(const motor_context_t *ctx)
{
    const uint8_t m = tl_open_seq_base();

    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_lq_uH();
    }
    if (tl_ident_l_sticky() != 0u) {
        return dbg.ld_lq_proc_psi_du_wb;
    }
    if (tl_ident_l_overlay() != 0u) {
        if ((m == 58u) || (m == 59u)) {
            return dbg.ld_lq_proc_psi_du_wb;
        }
        return dbg.ld_lq_proc_code;
    }
    if ((m >= 1u) && (m <= 30u)) {
        return (ctx != NULL) ? ctx->id : dbg.foc_id;
    }
    return dbg.foc_duty_tb;
}

/**
 * @brief IDENT ch10：L_avg_uH；Pass0=id_ref−id。
 */
static float tl_ident_ch10(const motor_context_t *ctx)
{
    const uint8_t m = tl_open_seq_base();
    float id_ref;
    float id_fb;

    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_lavg_uH();
    }
    if (tl_ident_l_sticky() != 0u) {
        return dbg.ld_lq_L_est_uH;
    }
    if (tl_ident_l_overlay() != 0u) {
        return dbg.ld_lq_L_est_uH;
    }
    if ((m >= 1u) && (m <= 30u)) {
        id_ref = (ctx != NULL) ? ctx->id_ref : dbg.foc_id_ref;
        id_fb = (ctx != NULL) ? ctx->id : dbg.foc_id;
        return id_ref - id_fb;
    }
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

#if M1_TELEM_BRINGUP_K <= 6u
/**
 * @brief 6ch@20kHz 紧凑辨识：id iq ud uq flex mark（表仍 12 槽，后 6 填 0）。
 * flex：L 段报 Lavg_uH 或 n_di；其余报 θe（锁轴检查）。
 * mark54 Rs：id iq ud_pi id_ref θe mark（供离线拟合斜率/截距）。
 * mark55 本轮 shout：Ld Lq Lavg round ok mark（连发，单位 µH）。
 */
static float tl_zero(const motor_context_t *ctx)
{
    (void)ctx;
    return 0.0f;
}

static uint8_t tl_is_rs_ramp(void)
{
    return (tl_open_seq_base() == 54u) ? 1u : 0u;
}

static float tl_ident6_ch0(const motor_context_t *ctx)
{
    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_ld_uH();
    }
    return tl_id(ctx);
}

static float tl_ident6_ch1(const motor_context_t *ctx)
{
    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_lq_uH();
    }
    return tl_iq(ctx);
}

static float tl_ident6_ch2(const motor_context_t *ctx)
{
    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_lavg_uH();
    }
    /* Rs 段挂 ud_pi（拟合用）；旧包 mark54 的 ud 几乎全 0 */
    if (tl_is_rs_ramp() != 0u) {
        return dbg.foc_ud_pi;
    }
    return tl_ident_ch6(ctx);
}

static float tl_ident6_ch3(const motor_context_t *ctx)
{
    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_round();
    }
    if (tl_is_rs_ramp() != 0u) {
        return (ctx != NULL) ? ctx->id_ref : dbg.foc_id_ref;
    }
    return tl_ident_ch7(ctx);
}

static float tl_ident_flex6(const motor_context_t *ctx)
{
    (void)ctx;
    if (l_hfi_ident_result_shout_active() != 0u) {
        return l_hfi_ident_shout_ok();
    }
    if (tl_is_rs_ramp() != 0u) {
        /* 跑中仍报 θe；Rs 算完后 dbg 已有 ohm/截距，但 mark 很快切走 */
        return dbg.foc_theta_el;
    }
    if (tl_ident_l_overlay() != 0u) {
        if ((dbg.ld_lq_ok != 0u) || (tl_ident_l_sticky() != 0u)) {
            return dbg.ld_lq_L_est_uH;
        }
        /* 跑中：上一圈有效 bin 数（应≈32） */
        return dbg.ld_lq_proc_half_ticks;
    }
    return dbg.foc_theta_el;
}

static const telem_ch_t s_layout_ident_duty[TELEM_TABLE_CH_N] = {
    { "id",    tl_ident6_ch0 },
    { "iq",    tl_ident6_ch1 },
    { "u_d",   tl_ident6_ch2 },
    { "u_q",   tl_ident6_ch3 },
    { "flex",  tl_ident_flex6 },
    { "mark",  tl_mark },
    { "z6",    tl_zero },
    { "z7",    tl_zero },
    { "z8",    tl_zero },
    { "z9",    tl_zero },
    { "z10",   tl_zero },
    { "z11",   tl_zero },
};
#else
/** 辨识：ia ib ic id iq θ；ch6/7 电压；ch8-10 随 open_seq 切 Pass0/L 结果 */
static const telem_ch_t s_layout_ident_duty[TELEM_TABLE_CH_N] = {
    { "ia",       tl_ia },
    { "ib",       tl_ib },
    { "ic",       tl_ic },
    { "id",       tl_id },
    { "iq",       tl_iq },
    { "theta_el", tl_theta_el },
    { "u_d",      tl_ident_ch6 },
    { "u_q",      tl_ident_ch7 },
    { "ident_a",  tl_ident_ch8 },
    { "ident_b",  tl_ident_ch9 },
    { "ident_c",  tl_ident_ch10 },
    { "mark",     tl_mark },
};
#endif

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
    /* 布局由配方宏在编译期常量化；写成 if 便于少 #if，不新增开关宏。 */
    if (M1_VOFA_OPT12) {
        return TELEM_LAYOUT_OPT;
    }
    if (M1_VOFA_SIGNOFF_CH) {
        return TELEM_LAYOUT_SIGNOFF;
    }
    if (M1_VOFA_IDENT_DUTY_12CH && !M1_SPEED_LOOP_ENABLE) {
        return TELEM_LAYOUT_IDENT_DUTY;
    }
    if (M1_SPEED_LOOP_ENABLE && M1_PLL_ENABLE) {
        return TELEM_LAYOUT_SPEED;
    }
    return TELEM_LAYOUT_FOC_DQ;
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
    uint8_t id = layout_id;

    if ((id != TELEM_LAYOUT_LEGACY) && (telem_table_find(id) == NULL)) {
        id = telem_table_default_layout();
    }
    s_layout = id;
    /* 旧 layout API → obs 预设（仅 RTOS 路径调用） */
    if (id != TELEM_LAYOUT_LEGACY) {
        (void)obs_preset_apply(id, 0u);
    }
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
    /* 实发通道数可能是 6；只填前 K 路，避免 6 元缓冲被写爆 */
    const uint32_t n_fill =
        (M1_TELEM_BRINGUP_K < TELEM_TABLE_CH_N) ? M1_TELEM_BRINGUP_K
                                                 : TELEM_TABLE_CH_N;

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

    for (i = 0u; i < n_fill; i++) {
        vals[i] = ch[i].get(ctx);
    }
    return 1u;
}
