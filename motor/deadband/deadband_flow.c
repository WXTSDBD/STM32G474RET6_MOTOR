/**
 * @file deadband_flow.c
 * @date 2026-10-06
 * @brief 配方调度：Rs→l_hfi（id_cal）或 ident。
 *
 * 节拍限制见 deadband_flow.h 文件头。
 * 能力闸门用 if (M1_*)；boot 时按宏拼 recipe，无 #if 分岔。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband_flow.h"

#include <stddef.h>

#include "deadband_id_cal.h"
#include "deadband_service.h"
#include "ident_flow.h"
#include "ident_module.h"
#include "motor_foc_loop.h"
#include "motor_params_m1.h"

typedef enum {
    DEADBAND_FLOW_KIND_ID_CAL = 0,
    DEADBAND_FLOW_KIND_IDENT
} deadband_flow_kind_t;

typedef struct {
    deadband_flow_kind_t kind;
} deadband_flow_step_t;

static deadband_flow_step_t s_flow_recipe[2];
static uint8_t s_flow_recipe_len;
static uint8_t s_flow_step;
static uint8_t s_ident_pi_retune_pending;
static uint8_t s_ident_done_handoff;

static void deadband_flow_rebuild_recipe(void)
{
    s_flow_recipe_len = 0u;
    if (M1_ID_LOCK_CAL_SWEEP) {
        s_flow_recipe[s_flow_recipe_len].kind = DEADBAND_FLOW_KIND_ID_CAL;
        s_flow_recipe_len++;
    }
    if (M1_IDENT_ENABLE) {
        s_flow_recipe[s_flow_recipe_len].kind = DEADBAND_FLOW_KIND_IDENT;
        s_flow_recipe_len++;
    }
}

static void deadband_flow_enter_step(motor_context_t *ctx, uint8_t step_idx)
{
    if (step_idx >= s_flow_recipe_len) {
        return;
    }

    switch (s_flow_recipe[step_idx].kind) {
    case DEADBAND_FLOW_KIND_ID_CAL:
        ctx->mode = M1_CTRL_CURRENT_LOOP;
        ctx->id_ref = 0.0f;
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
        motor_foc_loop_pi_reset(ctx);
        deadband_id_cal_init();
        deadband_id_cal_boot_rs_l_hfi_only(ctx);
        break;

    case DEADBAND_FLOW_KIND_IDENT:
        if (M1_IDENT_ENABLE) {
            s_ident_pi_retune_pending = 1u;
            s_ident_done_handoff = 0u;
            ident_flow_init(ctx);
        }
        break;

    default:
        break;
    }
}

void deadband_flow_boot(motor_context_t *ctx)
{
    if ((!M1_DEADBAND_FLOW_ENABLE) || (ctx == NULL)) {
        return;
    }

    deadband_flow_rebuild_recipe();
    if (s_flow_recipe_len == 0u) {
        return;
    }

    s_flow_step = 0u;
    s_ident_pi_retune_pending = 0u;
    s_ident_done_handoff = 0u;
    deadband_flow_enter_step(ctx, 0u);
}

void deadband_flow_tick(motor_context_t *ctx)
{
    if ((!M1_DEADBAND_FLOW_ENABLE) || (ctx == NULL) ||
        (s_flow_recipe_len == 0u) || (s_flow_step >= s_flow_recipe_len)) {
        return;
    }

    switch (s_flow_recipe[s_flow_step].kind) {
    case DEADBAND_FLOW_KIND_ID_CAL:
        deadband_id_cal_tick(ctx);
        if (deadband_id_cal_is_done()) {
            s_flow_step++;
            if (s_flow_step < s_flow_recipe_len) {
                deadband_flow_enter_step(ctx, s_flow_step);
            }
        }
        break;

    case DEADBAND_FLOW_KIND_IDENT:
        if (M1_IDENT_ENABLE) {
            ident_flow_tick(ctx);
            if (s_ident_done_handoff == 0u &&
                ident_module_get_state() == IDENT_MOD_DONE) {
                ctx->id_ref = 0.0f;
                ctx->iq_ref = M1_IDENT_STEP_I0_A;
                s_ident_done_handoff = 1u;
                s_flow_step++;
                if (s_flow_step < s_flow_recipe_len) {
                    deadband_flow_enter_step(ctx, s_flow_step);
                }
            }
        }
        break;

    default:
        break;
    }
}

uint8_t deadband_flow_id_cal_active(void)
{
    if ((!M1_DEADBAND_FLOW_ENABLE) || (!M1_ID_LOCK_CAL_SWEEP)) {
        return 0u;
    }
    if ((s_flow_recipe_len == 0u) || (s_flow_step >= s_flow_recipe_len)) {
        return 0u;
    }
    if (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_ID_CAL) {
        return deadband_id_cal_is_running();
    }
    return 0u;
}

uint8_t deadband_flow_consume_ident_pi_retune(void)
{
    const uint8_t pending = s_ident_pi_retune_pending;

    s_ident_pi_retune_pending = 0u;
    return pending;
}

uint8_t deadband_flow_in_ident(void)
{
    if ((!M1_DEADBAND_FLOW_ENABLE) || (!M1_IDENT_ENABLE)) {
        return 0u;
    }
    if ((s_flow_recipe_len == 0u) || (s_flow_step >= s_flow_recipe_len)) {
        return 0u;
    }
    return (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_IDENT) ? 1u : 0u;
}
