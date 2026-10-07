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
 *   mark ← dbg.open_seq_phase（D 打点，保留）
 *
 * FOC_DQ / IDENT 仍有部分通道读 dbg（ud/uq/duty 等，本刀未迁）。
 */

#include "telem_table.h"

#include <stddef.h>

#include "motor_params_m1.h"
#include "dbg_monitor.h"
#include "motor_current.h"

#ifndef M1_VOFA_SIGNOFF_CH
#define M1_VOFA_SIGNOFF_CH 0
#endif
#ifndef M1_VOFA_IDENT_DUTY_12CH
#define M1_VOFA_IDENT_DUTY_12CH 0
#endif
#ifndef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11 0
#endif
#ifndef M1_POS_MIT_COMBO_ENABLE
#define M1_POS_MIT_COMBO_ENABLE 0
#endif
#ifndef M1_SPEED_LOOP_ENABLE
#define M1_SPEED_LOOP_ENABLE 0
#endif
#ifndef M1_PLL_ENABLE
#define M1_PLL_ENABLE 0
#endif

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

/** 速度环常用：前 8 同 foc_dq 的 ud/uq 段，后为 ω_pll ω_ref iq_ref mark */
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
    { "iq_ref",    tl_iq_ref_outer },
    { "mark",      tl_mark },
};

/** 辨识 duty：ia ib ic id iq θ vd vq ta tb tc mark */
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
#if M1_VOFA_SIGNOFF_CH
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
