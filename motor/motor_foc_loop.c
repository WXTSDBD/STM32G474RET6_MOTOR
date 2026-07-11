/**
 * @file motor_foc_loop.c
 * @brief M1 Id/Iq PI 编排（算法在 foc_pi.c）。
 */

#include "motor_foc_loop.h"

#include "deadband_cal.h"
#include "deadband_flow.h"
#include "deadband_id_cal.h"
#include "dbg_monitor.h"
#include "foc_pi.h"
#include "ld_lq_ident.h"
#include "motor_current.h"
#include "motor_params_m1.h"
#include "speed_ident_flow.h"

static float motor_foc_loop_clamp_ref(float ref)
{
    if (ref > M1_I_REF_ABS_MAX) {
        return M1_I_REF_ABS_MAX;
    }
    if (ref < -M1_I_REF_ABS_MAX) {
        return -M1_I_REF_ABS_MAX;
    }
    return ref;
}

#if M1_IDENT_ENABLE
static void motor_foc_loop_pi_apply_ident_limits(motor_context_t *ctx)
{
    foc_pi_init(&ctx->pi_id, M1_PI_KP_ID, M1_PI_KI,
                M1_IDENT_PI_V_LIMIT_MIN, M1_IDENT_PI_V_LIMIT_V,
                M1_IDENT_PI_INT_LIMIT_MIN, M1_IDENT_PI_INT_LIMIT_V);
    foc_pi_init(&ctx->pi_iq, M1_PI_KP_IQ, M1_PI_KI,
                M1_IDENT_PI_V_LIMIT_MIN, M1_IDENT_PI_V_LIMIT_V,
                M1_IDENT_PI_INT_LIMIT_MIN, M1_IDENT_PI_INT_LIMIT_V);
}
#endif

void motor_foc_loop_pi_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

#if M1_IDENT_ENABLE && M1_IDENT_ID_CAL_BEFORE_STEP
    foc_pi_init(&ctx->pi_id, M1_PI_KP_ID, M1_PI_KI,
                M1_ID_CAL_PI_V_LIMIT_MIN, M1_ID_CAL_PI_V_LIMIT_V,
                M1_ID_CAL_PI_INT_LIMIT_MIN, M1_ID_CAL_PI_INT_LIMIT_V);
    foc_pi_init(&ctx->pi_iq, M1_PI_KP_IQ, M1_PI_KI,
                M1_ID_CAL_PI_V_LIMIT_MIN, M1_ID_CAL_PI_V_LIMIT_V,
                M1_ID_CAL_PI_INT_LIMIT_MIN, M1_ID_CAL_PI_INT_LIMIT_V);
#elif M1_ID_LOCK_CAL_SWEEP
    foc_pi_init(&ctx->pi_id, M1_PI_KP_ID, M1_PI_KI,
                M1_ID_CAL_PI_V_LIMIT_MIN, M1_ID_CAL_PI_V_LIMIT_V,
                M1_ID_CAL_PI_INT_LIMIT_MIN, M1_ID_CAL_PI_INT_LIMIT_V);
    foc_pi_init(&ctx->pi_iq, M1_PI_KP_IQ, M1_PI_KI,
                M1_ID_CAL_PI_V_LIMIT_MIN, M1_ID_CAL_PI_V_LIMIT_V,
                M1_ID_CAL_PI_INT_LIMIT_MIN, M1_ID_CAL_PI_INT_LIMIT_V);
#elif M1_IDENT_ENABLE && M1_IDENT_OVERRIDE_LIMITS
    foc_pi_init(&ctx->pi_id, M1_PI_KP_ID, M1_PI_KI,
                M1_IDENT_PI_V_LIMIT_MIN, M1_IDENT_PI_V_LIMIT_V,
                M1_IDENT_PI_INT_LIMIT_MIN, M1_IDENT_PI_INT_LIMIT_V);
    foc_pi_init(&ctx->pi_iq, M1_PI_KP_IQ, M1_PI_KI,
                M1_IDENT_PI_V_LIMIT_MIN, M1_IDENT_PI_V_LIMIT_V,
                M1_IDENT_PI_INT_LIMIT_MIN, M1_IDENT_PI_INT_LIMIT_V);
#else
    foc_pi_init(&ctx->pi_id, M1_PI_KP_ID, M1_PI_KI,
                M1_PI_V_LIMIT_MIN, M1_PI_V_LIMIT_V,
                M1_PI_INT_LIMIT_MIN, M1_PI_INT_LIMIT_V);
    foc_pi_init(&ctx->pi_iq, M1_PI_KP_IQ, M1_PI_KI,
                M1_PI_V_LIMIT_MIN, M1_PI_V_LIMIT_V,
                M1_PI_INT_LIMIT_MIN, M1_PI_INT_LIMIT_V);
#endif
}

void motor_foc_loop_pi_reset(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
}

void motor_foc_loop_on_flow_tick(motor_context_t *ctx)
{
#if M1_IDENT_ENABLE && M1_IDENT_OVERRIDE_LIMITS
    if (deadband_flow_consume_ident_pi_retune()) {
        motor_foc_loop_pi_apply_ident_limits(ctx);
    }
#else
    (void)ctx;
#endif
}

void motor_foc_loop_dbg_id_ref(const motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
#if M1_ID_LOCK_CAL_SWEEP
        if (deadband_flow_speed_ladder_active()) {
            dbg.foc_id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
            dbg.foc_iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
        } else if (deadband_id_cal_use_cal_pi_limits()) {
            dbg.foc_id_ref = deadband_id_cal_clamp_id_ref(ctx->id_ref);
            dbg.foc_iq_ref = deadband_id_cal_clamp_id_ref(ctx->iq_ref);
        } else {
            dbg.foc_id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
            dbg.foc_iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
        }
#else
        dbg.foc_id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
        dbg.foc_iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
#endif
    } else {
        dbg.foc_id_ref = 0.0f;
        dbg.foc_iq_ref = 0.0f;
    }
}

void motor_foc_loop_tick(motor_context_t *ctx,
                         float id,
                         float iq,
                         motor_startup_step_t *startup,
                         float theta_enc_park)
{
    float id_ref;
    float iq_ref;

    if (ctx == NULL || startup == NULL) {
        return;
    }

#if M1_ID_LOCK_CAL_SWEEP
    if (deadband_id_cal_bumpless_arm()) {
        const float ud_ff = M1_RS_OHM * deadband_id_cal_bumpless_id_ref();

        foc_pi_bumpless(&ctx->pi_id, ud_ff, deadband_id_cal_bumpless_id_ref(), id);
        ctx->ud_pi = ud_ff;
        if (!deadband_id_cal_iq_bumpless_arm()) {
            foc_pi_bumpless(&ctx->pi_iq, 0.0f, ctx->iq_ref, iq);
            ctx->uq_pi = 0.0f;
        }
        deadband_id_cal_consume_bumpless_arm();
    }
    if (deadband_id_cal_iq_bumpless_arm()) {
        foc_pi_bumpless(&ctx->pi_iq, ctx->uq_pi, deadband_id_cal_iq_bumpless_ref(), iq);
        deadband_id_cal_consume_iq_bumpless_arm();
    }
#endif

#if M1_ID_LOCK_CAL_SWEEP
    if (deadband_flow_id_cal_active()) {
        uint8_t id_cal_pi_run = 1u;

#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
        if (deadband_id_cal_use_align_ud()) {
            id_cal_pi_run = 0u;
        }
#endif
#if M1_LD_LQ_IDENT_ENABLE
        if (deadband_id_cal_use_ld_lq_align_ud()) {
            id_cal_pi_run = 0u;
        }
#endif
        if (ctx->mode == M1_CTRL_CURRENT_LOOP && !startup->use_fixed_uq &&
            (!deadband_id_cal_bumpless_arm() ||
             (deadband_id_cal_in_ld_lq_ident() && ld_lq_ident_pi_active()) ||
             deadband_id_cal_in_rs_ident() ||
             deadband_id_cal_in_ld_lq_pre_decay()) &&
            id_cal_pi_run) {
            if (deadband_id_cal_in_iq_probe()) {
                iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
#if M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE
                id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
                ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
#else
                id_ref = 0.0f;
                ctx->ud_pi = 0.0f;
#endif
                ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
            } else if (deadband_id_cal_use_cal_pi_limits()) {
                id_ref = deadband_id_cal_clamp_id_ref(ctx->id_ref);
                iq_ref = deadband_id_cal_clamp_iq_ref(ctx->iq_ref);
                if (startup->pi_bumpless) {
                    foc_pi_bumpless(&ctx->pi_iq, startup->uq_prev, iq_ref, iq);
                    foc_pi_bumpless(&ctx->pi_id, startup->ud_prev, id_ref, id);
                }
                ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
                ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
            } else {
                id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
                iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
                if (startup->pi_bumpless) {
                    foc_pi_bumpless(&ctx->pi_iq, startup->uq_prev, iq_ref, iq);
                    foc_pi_bumpless(&ctx->pi_id, startup->ud_prev, id_ref, id);
                }
                ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
                ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
            }
        } else if (!deadband_flow_id_cal_active()) {
            /* fall through to ident PI below */
#if M1_LD_LQ_IDENT_ENABLE && M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
        } else if (deadband_id_cal_in_ld_lq_ident() && ld_lq_ident_inject_active()) {
            ctx->ud_pi = ld_lq_ident_u_bias_d();
            ctx->uq_pi = ld_lq_ident_u_bias_q();
#endif
        } else {
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
        }
    }
    if (!deadband_flow_id_cal_active())
#endif
    {
#if M1_IDENT_ENABLE
        if (ctx->mode == M1_CTRL_CURRENT_LOOP && !startup->use_fixed_uq) {
            id_ref = ctx->id_ref;
            iq_ref = ctx->iq_ref;
            if (startup->pi_bumpless) {
                foc_pi_bumpless(&ctx->pi_iq, startup->uq_prev, iq_ref, iq);
                foc_pi_bumpless(&ctx->pi_id, startup->ud_prev, id_ref, id);
            }
            ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
            ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
        } else {
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
        }
#elif M1_ID_LOCK_CAL_SWEEP
#if M1_DEADBAND_FLOW_ONE_SHOT && M1_SPEED_LOOP_ENABLE
        if (deadband_flow_speed_ladder_active() &&
            ctx->mode == M1_CTRL_CURRENT_LOOP && !startup->use_fixed_uq) {
            id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
            iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
            if (startup->pi_bumpless) {
                foc_pi_bumpless(&ctx->pi_iq, startup->uq_prev, iq_ref, iq);
                foc_pi_bumpless(&ctx->pi_id, startup->ud_prev, id_ref, id);
            }
            ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
            ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
        } else {
            /* Id 标定段：由 deadband_flow_id_cal_active 分支处理 */
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
        }
#else
        /* pure id cal: handled above when deadband_flow_id_cal_active() */
        ctx->ud_pi = 0.0f;
        ctx->uq_pi = 0.0f;
#endif
#else
        if (ctx->mode == M1_CTRL_CURRENT_LOOP && !startup->use_fixed_uq) {
            id_ref = motor_foc_loop_clamp_ref(ctx->id_ref);
#if M1_SPEED_IDENT_ENABLE
            if (speed_ident_flow_is_armed() ||
                ctx->outer_mode == M1_OUTER_SPEED ||
                ctx->outer_mode == M1_OUTER_POSITION) {
                iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
            } else
#endif
#if M1_SPEED_LOOP_ENABLE && !M1_SPEED_IDENT_ENABLE
            if (ctx->outer_mode == M1_OUTER_SPEED ||
                ctx->outer_mode == M1_OUTER_POSITION) {
                iq_ref = motor_foc_loop_clamp_ref(ctx->iq_ref);
            } else
#endif
            {
                iq_ref = motor_foc_loop_clamp_ref(startup->iq_ref);
            }
            if (startup->pi_bumpless) {
                foc_pi_bumpless(&ctx->pi_iq, startup->uq_prev, iq_ref, iq);
                foc_pi_bumpless(&ctx->pi_id, startup->ud_prev, id_ref, id);
            }
            ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
            ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
        } else {
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
        }
#endif
    }
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
    if (deadband_id_cal_use_align_ud()) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
        ctx->ud_pi = deadband_id_cal_align_ud_v();
#else
        ctx->ud_pi = M1_ID_CAL_ALIGN_UD_V;
#endif
        ctx->uq_pi = 0.0f;
    } else if (deadband_id_cal_use_post_ident_hold_ud()) {
        ctx->ud_pi = M1_ID_CAL_ALIGN_UD_V;
        ctx->uq_pi = 0.0f;
    }
#endif

#if M1_FOC_ROTATION_FF_ENABLE && M1_SPEED_LOOP_ENABLE
    if ((ctx->outer_mode == M1_OUTER_SPEED ||
         ctx->outer_mode == M1_OUTER_POSITION) &&
        ctx->mode == M1_CTRL_CURRENT_LOOP &&
        !startup->use_fixed_uq) {
#if M1_ID_LOCK_CAL_SWEEP
        if (!deadband_flow_id_cal_active())
#endif
        {
            float omega_mech_rpm;
            float omega_e;

#if M1_PLL_ENABLE
            omega_mech_rpm = motor_current_get_pll_omega_mech_rpm();
#else
            omega_mech_rpm = 0.0f;
#endif
            /* ω_e [rad/s] = ω_mech [rpm] × 2π/60 × pole_pairs */
            omega_e = omega_mech_rpm * (0.10471975512f * (float)M1_POLE_PAIRS);
            ctx->ud_pi += M1_RS_OHM * ctx->id_ref - omega_e * M1_LD_H * ctx->iq_ref;
            ctx->uq_pi += M1_RS_OHM * ctx->iq_ref + omega_e * M1_LQ_H * ctx->id_ref;
        }
    }
#endif

    dbg.foc_ud_pi = ctx->ud_pi;
    dbg.foc_uq_pi = startup->use_fixed_uq ? startup->uq_out : ctx->uq_pi;

#if M1_ID_LOCK_CAL_SWEEP
    if (deadband_flow_id_cal_active() &&
        deadband_id_cal_should_capture() &&
        ctx->mode == M1_CTRL_CURRENT_LOOP &&
        !startup->use_fixed_uq) {
        if (deadband_cal_capture_at(id, ctx->ud_pi, deadband_id_cal_capture_id_ref(),
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
                                    deadband_id_cal_target_theta(),
                                    deadband_id_cal_capture_append_dlut())) {
#elif M1_ID_CAL_FIX_THETA_ENABLE
                                    M1_ID_CAL_THETA_EL_RAD, 1u)) {
#else
                                    theta_enc_park, 1u)) {
#endif
            deadband_id_cal_clear_capture_pending();
        }
    }
    if (deadband_flow_id_cal_active()) {
        deadband_id_cal_sync_dbg();
    }
#else
    (void)theta_enc_park;
#endif
}
