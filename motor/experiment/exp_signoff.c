/**
 * @file exp_signoff.c
 * @date 2026-10-07
 * @brief 签收 9 段 const 表 + SCRIPT 包装 motor_outer_signoff_*（E0 逐拍等价）。
 *
 * 段内激励仍由现网 outer_sign_seg_* 推进；本文件只登记段基 mark 与实验入口。
 * PACK1 跳段逻辑仍在 outer 内，不在此重写。
 */

#include "exp_signoff.h"

#include <stddef.h>

#include "motor_outer_loop.h"
#include "motor_params_m1.h"

#ifndef TELEM_LAYOUT_SIGNOFF
#define TELEM_LAYOUT_SIGNOFF  1u
#endif

#if M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE && \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)

/**
 * 9 段签收表（与 outer OUTER_SIGN_SEG_* / s_sign_seq_base 对齐）。
 * dur_ms=0：时长由 SCRIPT（现网段逻辑）决定。
 */
static const exp_seg_t s_signoff_segs[] = {
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 10u,  0u }, /* STEP */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 45u,  0u }, /* SINE */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 60u,  0u }, /* TRAP */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 70u,  0u }, /* DIST */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 210u, 0u }, /* PREV */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 230u, 0u }, /* MREV */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 80u,  0u }, /* MK */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 95u,  0u }, /* MSINE */
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 150u, 0u }, /* REL */
};

static const exp_cfg_t s_signoff_cfg[] = {
    { 0u, 0u, 0u, 0u, 0.0f, 0.0f },
};

static void exp_signoff_script_init(motor_context_t *ctx, const exp_cfg_t *cfg)
{
    (void)cfg;
    motor_outer_signoff_arm(ctx);
}

static void exp_signoff_script_tick(motor_context_t *ctx,
                                    float theta_fb_rad,
                                    float omega_rpm)
{
    motor_outer_signoff_tick(ctx, theta_fb_rad, omega_rpm);
}

static uint8_t exp_signoff_script_done(const motor_context_t *ctx)
{
    (void)ctx;
    return (motor_outer_signoff_is_armed() == 0u) ? 1u : 0u;
}

static const exp_script_ops_t s_signoff_script = {
    exp_signoff_script_init,
    exp_signoff_script_tick,
    exp_signoff_script_done,
};

const exp_def_t g_exp_signoff = {
    EXP_ID_SIGNOFF,
    "signoff_pos_mit",
    EXP_KIND_SCRIPT,
    s_signoff_segs,
    (uint16_t)(sizeof(s_signoff_segs) / sizeof(s_signoff_segs[0])),
    s_signoff_cfg,
    1u,
    TELEM_LAYOUT_SIGNOFF,
    &s_signoff_script,
};

#else /* 签收未编入：占位，避免空引用 */

static const exp_seg_t s_signoff_segs_dummy[1] = {
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 0u, 0u },
};

const exp_def_t g_exp_signoff = {
    EXP_ID_SIGNOFF,
    "signoff_pos_mit",
    EXP_KIND_SCRIPT,
    s_signoff_segs_dummy,
    0u,
    NULL,
    0u,
    0u,
    NULL,
};

#endif
