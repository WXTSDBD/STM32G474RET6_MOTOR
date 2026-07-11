/**
 * @file deadband_flow.c
 * @brief 配方表 + 步骤调度。
 */

#include "deadband_flow.h"

#include <stddef.h>

#include "deadband_id_cal.h"
#include "ident_flow.h"
#include "ident_module.h"
#include "speed_ident_flow.h"
#include "speed_ident_module.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_outer_loop.h"
#include "deadband_module.h"
#include "deadband_cal.h"
#include "dbg_monitor.h"
#include "motor_params_m1.h"
#if M1_VOFA_LUT_DUMP_ENABLE
#include "telem_lut_dump.h"
#endif

#if M1_DEADBAND_FLOW_ENABLE

typedef enum {
    DEADBAND_FLOW_KIND_ID_CAL = 0,
    DEADBAND_FLOW_KIND_IDENT,
#if M1_SPEED_IDENT_ENABLE
    DEADBAND_FLOW_KIND_SPEED_IDENT,
#endif
#if M1_DEADBAND_FLOW_ONE_SHOT
    DEADBAND_FLOW_KIND_SPEED_OFF,
    DEADBAND_FLOW_KIND_SPEED_LUT,
#endif
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE
    DEADBAND_FLOW_KIND_OPEN_UQ_LADDER,
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
    DEADBAND_FLOW_KIND_OPEN_UD_LADDER,
#endif
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && M1_LD_LQ_IDENT_ENABLE
    DEADBAND_FLOW_KIND_LD_LQ_IDENT,
#endif
#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    DEADBAND_FLOW_KIND_OPEN_UQ,
#endif
} deadband_flow_kind_t;

typedef struct {
    deadband_flow_kind_t kind;
} deadband_flow_step_t;

static const deadband_flow_step_t s_flow_recipe[] = {
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE
    { DEADBAND_FLOW_KIND_OPEN_UQ_LADDER },
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE && !M1_OPEN_UD_AFTER_ID_CAL
    { DEADBAND_FLOW_KIND_OPEN_UD_LADDER },
#endif
#if M1_ID_LOCK_CAL_SWEEP && (!M1_IDENT_ENABLE || M1_IDENT_ID_CAL_BEFORE_STEP)
    { DEADBAND_FLOW_KIND_ID_CAL },
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE && M1_OPEN_UD_AFTER_ID_CAL
    { DEADBAND_FLOW_KIND_OPEN_UD_LADDER },
#endif
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && M1_LD_LQ_IDENT_ENABLE
    { DEADBAND_FLOW_KIND_LD_LQ_IDENT },
#endif
#if M1_DEADBAND_FLOW_ONE_SHOT
    { DEADBAND_FLOW_KIND_SPEED_OFF },
    { DEADBAND_FLOW_KIND_SPEED_LUT },
#endif
#if M1_IDENT_ENABLE
    { DEADBAND_FLOW_KIND_IDENT },
#endif
#if M1_SPEED_IDENT_ENABLE
    { DEADBAND_FLOW_KIND_SPEED_IDENT },
#endif
#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    { DEADBAND_FLOW_KIND_OPEN_UQ },
#endif
};

#define DEADBAND_FLOW_RECIPE_LEN  (sizeof(s_flow_recipe) / sizeof(s_flow_recipe[0]))

static uint8_t s_flow_step;
static uint8_t s_ident_pi_retune_pending;
static uint8_t s_ident_done_handoff;
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
static uint8_t s_ladder_done_handoff;
#endif

static void deadband_flow_enter_open_pre_id_ladder(motor_context_t *ctx)
{
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    ctx->uq_open = 0.0f;
    ctx->ud_open = 0.0f;
    motor_foc_loop_pi_reset(ctx);
    ctx->mode = M1_CTRL_OPEN_LOOP;
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#elif M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
    deadband_service_apply_profile(DEADBAND_PROFILE_LUT_RUNTIME);
#else
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#endif
}

#if M1_DEADBAND_FLOW_ONE_SHOT && M1_SPEED_LOOP_ENABLE
static void deadband_flow_enter_speed_off(motor_context_t *ctx)
{
    ctx->mode = M1_CTRL_CURRENT_LOOP;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    ctx->uq_open = 0.0f;
    ctx->ud_open = 0.0f;
    motor_foc_loop_pi_reset(ctx);
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    dbg.deadband_mode = (uint8_t)deadband_service_get_mode();
#if M1_SPEED_PROFILE_ENABLE
    motor_speed_profile_arm_ex(ctx, 0u);
    motor_outer_set_mode(ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
    dbg.open_seq_phase = 200u;
}

static void deadband_flow_enter_speed_lut(motor_context_t *ctx)
{
    ctx->mode = M1_CTRL_CURRENT_LOOP;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    motor_foc_loop_pi_reset(ctx);
    if (deadband_cal_len() >= 2u) {
        deadband_cal_switch_runtime_lut(0u);
    } else {
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    }
    dbg.deadband_mode = (uint8_t)deadband_service_get_mode();
#if M1_SPEED_PROFILE_ENABLE
    motor_speed_profile_arm_ex(ctx, (M1_SPEED_PROFILE_REPEAT != 0) ? 1u : 0u);
    motor_outer_set_mode(ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
    dbg.open_seq_phase = 210u;
}
#endif /* M1_DEADBAND_FLOW_ONE_SHOT */

static void deadband_flow_enter_step(motor_context_t *ctx,
                                     uint8_t step_idx,
                                     uint8_t from_id_cal_handoff)
{
    if (step_idx >= DEADBAND_FLOW_RECIPE_LEN) {
        return;
    }

    switch (s_flow_recipe[step_idx].kind) {
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UQ_LADDER:
        deadband_flow_enter_open_pre_id_ladder(ctx);
        motor_open_sweep_begin_pre_id_uq_ladder(ctx);
        break;
#endif

#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UD_LADDER:
        deadband_flow_enter_open_pre_id_ladder(ctx);
        if (from_id_cal_handoff != 0u) {
            s_ladder_done_handoff = 0u;
        }
        motor_open_sweep_begin_pre_id_ud_ladder(ctx);
        break;
#endif

#if M1_ID_LOCK_CAL_SWEEP
    case DEADBAND_FLOW_KIND_ID_CAL:
        ctx->mode = M1_CTRL_CURRENT_LOOP;
        ctx->uq_open = 0.0f;
        ctx->ud_open = 0.0f;
        ctx->id_ref = 0.0f;
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
        motor_foc_loop_pi_reset(ctx);
#if M1_SPEED_LOOP_ENABLE
        motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
#endif
        deadband_id_cal_init();
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
        deadband_id_cal_boot_iq_probe_only(ctx);
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && !M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
        deadband_id_cal_boot_rs_ld_lq_only(ctx);
#endif
#if M1_DEADBAND_FLOW_ONE_SHOT
        dbg.open_seq_phase = 201u;
#endif
        break;
#endif

#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && M1_LD_LQ_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_LD_LQ_IDENT:
        ctx->mode = M1_CTRL_CURRENT_LOOP;
        ctx->uq_open = 0.0f;
        ctx->ud_open = 0.0f;
        ctx->id_ref = 0.0f;
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
        motor_foc_loop_pi_reset(ctx);
#if M1_SPEED_LOOP_ENABLE
        motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
#endif
        deadband_id_cal_reinit_ld_lq_chain(ctx);
        break;
#endif

#if M1_DEADBAND_FLOW_ONE_SHOT && M1_SPEED_LOOP_ENABLE
    case DEADBAND_FLOW_KIND_SPEED_OFF:
        deadband_flow_enter_speed_off(ctx);
        break;

    case DEADBAND_FLOW_KIND_SPEED_LUT:
        deadband_flow_enter_speed_lut(ctx);
        break;
#endif

#if M1_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_IDENT:
        ident_flow_init(ctx);
#if M1_IDENT_OVERRIDE_LIMITS
        if (from_id_cal_handoff != 0u) {
            s_ident_pi_retune_pending = 1u;
        }
#endif
        break;
#endif

#if M1_SPEED_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_SPEED_IDENT:
        ctx->mode = M1_CTRL_CURRENT_LOOP;
        ctx->id_ref = 0.0f;
        ctx->iq_ref = 0.0f;
        ctx->uq_open = 0.0f;
        ctx->ud_open = 0.0f;
        /* speed_ident_flow_init() 延后到 motor_current_init：PLL/编码器就绪后 */
        break;
#endif

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
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
    s_ladder_done_handoff = 0u;
#endif
    deadband_flow_enter_step(ctx, 0u, 0u);
}

void deadband_flow_tick(motor_context_t *ctx)
{
    if (ctx == NULL || DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return;
    }

    switch (s_flow_recipe[s_flow_step].kind) {
#if M1_DEADBAND_FLOW_ONE_SHOT && M1_SPEED_LOOP_ENABLE
    case DEADBAND_FLOW_KIND_SPEED_OFF:
        if (motor_speed_profile_consume_ladder_done()) {
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step, 0u);
            }
        }
        break;

    case DEADBAND_FLOW_KIND_SPEED_LUT:
        /* ③ 段：持续跑 profile（默认 repeat），用户停录 */
        break;
#endif

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UQ_LADDER:
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UD_LADDER:
#endif
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
        if (s_ladder_done_handoff == 0u && motor_open_sweep_done()) {
#if M1_VOFA_LUT_DUMP_ENABLE && M1_OPEN_UD_AFTER_ID_CAL
            telem_lut_dump_arm();
#endif
            s_ladder_done_handoff = 1u;
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step, 0u);
            }
        }
        break;
#endif

#if M1_ID_LOCK_CAL_SWEEP
    case DEADBAND_FLOW_KIND_ID_CAL:
        deadband_id_cal_tick(ctx);
        if (deadband_id_cal_is_done()) {
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step, 1u);
            }
        }
        break;
#endif

#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && M1_LD_LQ_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_LD_LQ_IDENT:
        deadband_id_cal_tick(ctx);
        if (deadband_id_cal_is_done()) {
            s_flow_step++;
            if (s_flow_step < DEADBAND_FLOW_RECIPE_LEN) {
                deadband_flow_enter_step(ctx, s_flow_step, 1u);
            }
        }
        break;
#endif

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
                deadband_flow_enter_step(ctx, s_flow_step, 0u);
            }
        }
        break;
#endif

#if M1_SPEED_IDENT_ENABLE
    case DEADBAND_FLOW_KIND_SPEED_IDENT:
        /* ω_ref 在 motor_outer_loop_tick @ 2 kHz 推进 */
        break;
#endif

#if M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
    case DEADBAND_FLOW_KIND_OPEN_UQ:
        if (!motor_open_sweep_active()) {
            ctx->mode = M1_CTRL_OBSERVE_ONLY;
            ctx->uq_open = 0.0f;
            ctx->ud_open = 0.0f;
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
    if (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_ID_CAL) {
        return deadband_id_cal_is_running();
    }
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && M1_LD_LQ_IDENT_ENABLE
    if (s_flow_recipe[s_flow_step].kind == DEADBAND_FLOW_KIND_LD_LQ_IDENT) {
        return deadband_id_cal_is_running();
    }
#endif
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

#if M1_DEADBAND_FLOW_ONE_SHOT && M1_SPEED_LOOP_ENABLE
uint8_t deadband_flow_speed_ladder_active(void)
{
    if (DEADBAND_FLOW_RECIPE_LEN == 0u ||
        s_flow_step >= DEADBAND_FLOW_RECIPE_LEN) {
        return 0u;
    }
    switch (s_flow_recipe[s_flow_step].kind) {
    case DEADBAND_FLOW_KIND_SPEED_OFF:
    case DEADBAND_FLOW_KIND_SPEED_LUT:
        return 1u;
    default:
        return 0u;
    }
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
           1u :
           0u;
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

#endif
