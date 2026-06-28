/**
 * @file deadband_flow.c
 * @brief 配方表 + 步骤调度。
 */

#include "deadband_flow.h"

#include <stddef.h>

#include "deadband_id_cal.h"
#include "ident_flow.h"
#include "ident_module.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "deadband_module.h"
#include "dbg_monitor.h"
#include "motor_params_m1.h"

#if (M1_ID_LOCK_CAL_SWEEP || M1_IDENT_ENABLE)

typedef enum {
    DEADBAND_FLOW_KIND_ID_CAL = 0,
    DEADBAND_FLOW_KIND_IDENT,
#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    DEADBAND_FLOW_KIND_OPEN_UQ,
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
#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    { DEADBAND_FLOW_KIND_OPEN_UQ },
#endif
};

#define DEADBAND_FLOW_RECIPE_LEN  (sizeof(s_flow_recipe) / sizeof(s_flow_recipe[0]))

static uint8_t s_flow_step;
static uint8_t s_ident_pi_retune_pending;
static uint8_t s_ident_done_handoff;

static void deadband_flow_enter_step(motor_context_t *ctx,
                                     uint8_t step_idx,
                                     uint8_t from_id_cal_handoff)
{
    if (step_idx >= DEADBAND_FLOW_RECIPE_LEN) {
        return;
    }

    switch (s_flow_recipe[step_idx].kind) {
    case DEADBAND_FLOW_KIND_ID_CAL:
        ctx->id_ref = 0.0f;
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
        deadband_id_cal_init();
        break;

    case DEADBAND_FLOW_KIND_IDENT:
        ident_flow_init(ctx);
#if M1_IDENT_OVERRIDE_LIMITS
        if (from_id_cal_handoff != 0u) {
            s_ident_pi_retune_pending = 1u;
        }
#endif
        break;

#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UQ:
        ctx->id_ref = 0.0f;
        ctx->iq_ref = 0.0f;
        motor_foc_loop_pi_reset(ctx);
        motor_open_sweep_begin_fixed(ctx,
                                     M1_IDENT_POST_BODE_OPEN_UQ_V,
                                     M1_IDENT_POST_BODE_OPEN_UQ_S);
        ctx->mode = M1_CTRL_OPEN_LOOP;
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
        dbg.open_seq_phase = 80u;
        break;
#endif

    default:
        break;
    }
}

void deadband_flow_boot(motor_context_t *ctx)
{
    if (ctx == NULL || DEADBAND_FLOW_RECIPE_LEN == 0u) {
        return;
    }

    s_flow_step = 0u;
    s_ident_pi_retune_pending = 0u;
    s_ident_done_handoff = 0u;
    deadband_flow_enter_step(ctx, 0u, 0u);
}

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
                deadband_flow_enter_step(ctx, s_flow_step, 1u);
            }
        }
        break;

    case DEADBAND_FLOW_KIND_IDENT:
        ident_flow_tick(ctx);
        if (s_ident_done_handoff == 0u &&
            ident_module_get_state() == IDENT_MOD_DONE) {
            s_ident_done_handoff = 1u;
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step, 0u);
            }
        }
        break;

#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UQ:
        if (!motor_open_sweep_active()) {
            ctx->mode = M1_CTRL_OBSERVE_ONLY;
            ctx->uq_open = 0.0f;
            dbg.open_seq_phase = 81u;
        }
        break;
#endif

    default:
        break;
    }
}

uint8_t deadband_flow_id_cal_active(void)
{
#if M1_ID_LOCK_CAL_SWEEP
    if (DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return 0u;
    }
    if (s_flow_recipe[s_flow_step].kind != DEADBAND_FLOW_KIND_ID_CAL) {
        return 0u;
    }
    return deadband_id_cal_is_running();
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

#else /* !(M1_ID_LOCK_CAL_SWEEP || M1_IDENT_ENABLE) */

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

#endif
