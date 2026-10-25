/**
 * @file exp_runner.c
 * @date 2026-10-07
 * @brief 实验注册表与执行器（E0 SCRIPT + E3 SCHEDULE）。
 *
 * SCHEDULE：按 dur_ms 推进段表；cfg_idx 变化时 apply exp_cfg。
 * 禁止在本文件加 switch(seg->type) 膨胀——加动作另开文件。
 */

#include "exp_runner.h"

#include <stddef.h>

#include "exp_act_basic.h"
#include "exp_cfg.h"
#include "exp_tables.h"
#include "dbg_monitor.h"
#include "motor_outer_loop.h"
#include "motor_params_m1.h"

/** 外环 tick 标称周期，单位 ms（2 kHz）。 */
#define EXP_OUTER_TICK_MS  0.5f

static const exp_def_t *const g_experiments[] = {
    /* 2026-10-25 table-driven experiments (data in exp_tables.c) */
    &g_exp_rev141,
    &g_exp_ud_cal,
    &g_exp_speed_ladder,
    &g_exp_pos_step,
    &g_exp_mit_rev,
    &g_exp_speed_rev,
    &g_exp_uq_sweep,
    &g_exp_ipd_sweep,
    &g_exp_ident,
    &g_exp_bode,
    &g_exp_vasi,
    &g_exp_if_drag,
    &g_exp_deadband_lut,
    &g_exp_qkick,
};

static const exp_def_t *s_active;
static uint8_t s_armed;
static uint16_t s_seg_i;
static float s_seg_t_ms;
static uint8_t s_cfg_idx;

/**
 * @brief 按 id 查找注册表。
 * @param exp_id 实验 id。
 * @return 定义指针；未找到为 NULL。
 */
static const exp_def_t *exp_runner_find(uint8_t exp_id)
{
    uint32_t i;

    for (i = 0u; i < (sizeof(g_experiments) / sizeof(g_experiments[0])); i++) {
        if ((g_experiments[i] != NULL) && (g_experiments[i]->id == exp_id)) {
            return g_experiments[i];
        }
    }
    return NULL;
}

/**
 * @brief 取当前段要用的动作 ops（段内 ops 优先，否则按 act 查内建表）。
 */
static const exp_action_ops_t *exp_runner_seg_ops(const exp_seg_t *seg)
{
    const exp_action_ops_t *ops;

    if (seg == NULL) {
        return NULL;
    }
    if (seg->ops != NULL) {
        return seg->ops;
    }
    ops = exp_act_basic_lookup(seg->act);
    return ops;
}

/**
 * @brief 进入一段：按需 apply cfg，再调 enter。
 * @param force_cfg 1=首段强制 apply；0=仅 cfg_idx 变化时 apply。
 */
static void exp_runner_enter_seg(motor_context_t *ctx,
                                 const exp_seg_t *seg,
                                 uint8_t force_cfg)
{
    const exp_action_ops_t *ops;

    if ((ctx == NULL) || (seg == NULL) || (s_active == NULL)) {
        return;
    }

    if ((s_active->cfg != NULL) &&
        (s_active->cfg_n > 0u) &&
        (seg->cfg_idx < s_active->cfg_n) &&
        ((force_cfg != 0u) || (seg->cfg_idx != s_cfg_idx))) {
        exp_cfg_apply(ctx, &s_active->cfg[seg->cfg_idx]);
        s_cfg_idx = seg->cfg_idx;
    }

    DBG.open_seq_phase = seg->mark;

    ops = exp_runner_seg_ops(seg);
    if ((ops != NULL) && (ops->enter != NULL)) {
        ops->enter(ctx, seg);
    }
}

void exp_runner_init(void)
{
    s_active = NULL;
    s_armed = 0u;
    s_seg_i = 0u;
    s_seg_t_ms = 0.0f;
    s_cfg_idx = 0xFFu;
}

uint8_t exp_runner_select(uint8_t exp_id)
{
    const exp_def_t *def = exp_runner_find(exp_id);

    if (def == NULL) {
        s_active = NULL;
        s_armed = 0u;
        return 0u;
    }
    s_active = def;
    s_armed = 0u;
    s_seg_i = 0u;
    s_seg_t_ms = 0.0f;
    s_cfg_idx = 0xFFu;
    return 1u;
}

void exp_runner_arm(motor_context_t *ctx)
{
    if ((s_active == NULL) || (ctx == NULL)) {
        s_armed = 0u;
        return;
    }

    if ((s_active->kind == EXP_KIND_SCRIPT) &&
        (s_active->script != NULL) &&
        (s_active->script->init != NULL)) {
        const exp_cfg_t *cfg =
            (s_active->cfg_n > 0u) ? &s_active->cfg[0] : NULL;

        /* 签收等 SCRIPT 自管；不在此套用可能全 0 的 cfg 表。 */
        (void)cfg;
        s_active->script->init(ctx, cfg);
        s_armed = 1u;
        s_seg_i = 0u;
        s_seg_t_ms = 0.0f;
        s_cfg_idx = 0xFFu;
        return;
    }

    if ((s_active->kind == EXP_KIND_SCHEDULE) &&
        (s_active->seg != NULL) &&
        (s_active->seg_n > 0u)) {
        s_seg_i = 0u;
        s_seg_t_ms = 0.0f;
        s_cfg_idx = 0xFFu;
        s_armed = 1u;
        exp_runner_enter_seg(ctx, &s_active->seg[0], 1u);
        return;
    }

    s_armed = 0u;
}

void exp_runner_tick(motor_context_t *ctx, float theta_fb_rad, float omega_rpm)
{
    const exp_seg_t *seg;
    const exp_action_ops_t *ops;

    (void)theta_fb_rad;
    (void)omega_rpm;

    if ((s_armed == 0u) || (s_active == NULL) || (ctx == NULL)) {
        return;
    }

    if ((s_active->kind == EXP_KIND_SCRIPT) &&
        (s_active->script != NULL) &&
        (s_active->script->tick != NULL)) {
        s_active->script->tick(ctx, theta_fb_rad, omega_rpm);
        if ((s_active->script->done != NULL) &&
            (s_active->script->done(ctx) != 0u)) {
            s_armed = 0u;
        }
        return;
    }

    if ((s_active->kind != EXP_KIND_SCHEDULE) ||
        (s_active->seg == NULL) ||
        (s_seg_i >= s_active->seg_n)) {
        s_armed = 0u;
        return;
    }

    seg = &s_active->seg[s_seg_i];
    ops = exp_runner_seg_ops(seg);
    if ((ops != NULL) && (ops->update != NULL)) {
        ops->update(ctx, seg, s_seg_t_ms * 0.001f);
    }

    s_seg_t_ms += EXP_OUTER_TICK_MS;

    if ((seg->dur_ms > 0u) && (s_seg_t_ms >= (float)seg->dur_ms)) {
        s_seg_i++;
        s_seg_t_ms = 0.0f;
        if (s_seg_i >= s_active->seg_n) {
            s_armed = 0u;
            return;
        }
        exp_runner_enter_seg(ctx, &s_active->seg[s_seg_i], 0u);
    }
}

uint8_t exp_runner_is_armed(void)
{
#if M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE && \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)
    if ((s_active != NULL) &&
        (s_active->kind == EXP_KIND_SCRIPT) &&
        (s_active->id == EXP_ID_SIGNOFF)) {
        /* 与现网 s_sign_armed 同步：停表后 runner 也视为未武装 */
        return motor_outer_signoff_is_armed();
    }
#endif
    return s_armed;
}

uint8_t exp_runner_active_id(void)
{
    return (s_active != NULL) ? s_active->id : EXP_ID_NONE;
}

const exp_def_t *exp_runner_active(void)
{
    return s_active;
}
