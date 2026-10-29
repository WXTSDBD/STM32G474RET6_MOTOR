/**
 * @file deadband_flow.c
 * @date 2026-10-06
 * @brief 配方调度：当前只拉起 Rs→l_hfi（id_cal）。

 *
 * 节拍限制见 deadband_flow.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。建表 ONE_SHOT/LUT
 *       阶梯已删除；原文件见 `_scratch/archive_deadband_lut/`。
 */

#include "deadband_flow.h"

#include <stddef.h>

#include "deadband_id_cal.h"
#include "deadband_service.h"
#include "ident_flow.h"
#include "ident_module.h"
#include "motor_foc_loop.h"
#include "motor_params_m1.h"
#include "speed_ident_flow.h"
#include "speed_ident_module.h"

#if M1_DEADBAND_FLOW_ENABLE

typedef enum {
    DEADBAND_FLOW_KIND_ID_CAL = 0,
#if M1_IDENT_ENABLE
    DEADBAND_FLOW_KIND_IDENT,
#endif
#if M1_SPEED_IDENT_ENABLE
    DEADBAND_FLOW_KIND_SPEED_IDENT,
#endif
} deadband_flow_kind_t;

typedef struct {
    deadband_flow_kind_t kind;
} deadband_flow_step_t;

static const deadband_flow_step_t s_flow_recipe[] = {
#if M1_ID_LOCK_CAL_SWEEP && (!M1_IDENT_ENABLE || M1_IDENT_ID_CAL_BEFORE_STEP)
    { DEADBAND_FLOW_KIND_ID_CAL },
#endif
#if M1_IDENT_ENABLE
    { DEADBAND_FLOW_KIND_IDENT },
#endif
#if M1_SPEED_IDENT_ENABLE
    { DEADBAND_FLOW_KIND_SPEED_IDENT },
#endif
};

#define DEADBAND_FLOW_RECIPE_LEN  (sizeof(s_flow_recipe) / sizeof(s_flow_recipe[0]))

static uint8_t s_flow_step;
static uint8_t s_ident_pi_retune_pending;
static uint8_t s_ident_done_handoff;

static void deadband_flow_enter_step(motor_context_t *ctx, uint8_t step_idx)
{
    if (step_idx >= DEADBAND_FLOW_RECIPE_LEN) {
        return;
    }

    switch (s_flow_recipe[step_idx].kind) {
    case DEADBAND_FLOW_KIND_ID_CAL:
        ctx->mode = M1_CTRL_CURRENT_LOOP;
        ctx->id_ref = 0.0f;
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
        motor_foc_loop_pi_reset(ctx);
        deadband_id_cal_init();
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
        deadband_id_cal_boot_rs_l_hfi_only(ctx);
#endif
        break;

#if M1_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_IDENT:
        s_ident_pi_retune_pending = 1u;
        s_ident_done_handoff = 0u;
        ident_flow_init(ctx);
        break;
#endif

#if M1_SPEED_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_SPEED_IDENT:
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
        break;
#endif

    default:
        break;
    }
}

/**
 * @brief 上电进入配方第一步。
 */
void deadband_flow_boot(motor_context_t *ctx)
{
    if (ctx == NULL || DEADBAND_FLOW_RECIPE_LEN == 0u) {
        return;
    }

    s_flow_step = 0u;
    s_ident_pi_retune_pending = 0u;
    s_ident_done_handoff = 0u;
    deadband_flow_enter_step(ctx, 0u);
}

/**
 * @brief 推进当前配方步。
 */
void deadband_flow_tick(motor_context_t *ctx)
{
    if (ctx == NULL || DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return;
    }

    switch (s_flow_recipe[s_flow_step].kind) {
    case DEADBAND_FLOW_KIND_ID_CAL:
        deadband_id_cal_tick(ctx);
        if (deadband_id_cal_is_done()) {
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step);
            }
        }
        break;

#if M1_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_IDENT:
        ident_flow_tick(ctx);
        if (s_ident_done_handoff == 0u &&
            ident_module_get_state() == IDENT_MOD_DONE) {
            ctx->id_ref = 0.0f;
            ctx->iq_ref = M1_IDENT_STEP_I0_A;
            s_ident_done_handoff = 1u;
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step);
            }
        }
        break;
#endif

#if M1_SPEED_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_SPEED_IDENT:
        break;
#endif

    default:
        break;
    }
}

/**
 * @brief 1=Id/Rs/L 标定仍占用热路径。
 */
uint8_t deadband_flow_id_cal_active(void)
{
#if M1_ID_LOCK_CAL_SWEEP
    if (DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return 0u;
    }
    if (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_ID_CAL) {
        return deadband_id_cal_is_running();
    }
    return 0u;
#else
    return 0u;
#endif
}

uint8_t deadband_flow_consume_ident_pi_retune(void)
{
    const uint8_t pending = s_ident_pi_retune_pending;

    s_ident_pi_retune_pending = 0u;
    return pending;
}

#if M1_IDENT_ENABLE
uint8_t deadband_flow_in_ident(void)
{
    if (DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return 0u;
    }
    return (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_IDENT) ? 1u : 0u;
}
#endif

#if M1_SPEED_IDENT_ENABLE
uint8_t deadband_flow_speed_ident_active(void)
{
    if (speed_ident_flow_is_armed()) {
        return 1u;
    }
    if (DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return 0u;
    }
    return (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_SPEED_IDENT) ?
           1u : 0u;
}
#endif

#else /* !M1_DEADBAND_FLOW_ENABLE */

void deadband_flow_boot(motor_context_t *ctx)
{
    (void)ctx;
}

void deadband_flow_tick(motor_context_t *ctx)
{
    (void)ctx;
}

uint8_t deadband_flow_id_cal_active(void)
{
    return 0u;
}

uint8_t deadband_flow_consume_ident_pi_retune(void)
{
    return 0u;
}

#if M1_SPEED_IDENT_ENABLE
uint8_t deadband_flow_speed_ident_active(void)
{
    return speed_ident_flow_is_armed();
}
#endif

#endif /* M1_DEADBAND_FLOW_ENABLE */
