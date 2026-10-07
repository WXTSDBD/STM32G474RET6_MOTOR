/**
 * @file exp_runner.c
 * @date 2026-10-07
 * @brief 实验注册表与执行器（E0）。
 *
 * SCHEDULE 段推进留给后续；本刀只跑 SCRIPT（签收包装）。
 * 禁止在本文件加 switch(seg->type) 膨胀——加动作另开文件。
 */

#include "exp_runner.h"

#include <stddef.h>

#include "exp_signoff.h"
#include "motor_outer_loop.h"
#include "motor_params_m1.h"

static const exp_def_t *const g_experiments[] = {
    &g_exp_signoff,
};

static const exp_def_t *s_active;
static uint8_t s_armed;

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

void exp_runner_init(void)
{
    s_active = NULL;
    s_armed = 0u;
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

        s_active->script->init(ctx, cfg);
        s_armed = 1u;
        return;
    }

    /* SCHEDULE：E0 未接；武装失败不碰 ctx */
    s_armed = 0u;
}

void exp_runner_tick(motor_context_t *ctx, float theta_fb_rad, float omega_rpm)
{
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
