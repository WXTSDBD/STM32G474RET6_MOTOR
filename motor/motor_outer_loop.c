/**
 * @file motor_outer_loop.c
 * @brief M1 2kHz 外环调度实现：DISABLED / TORQUE / SPEED / POSITION + bumpless 切换。
 */

#include "motor_outer_loop.h"

#include <stddef.h>

#include "dbg_monitor.h"
#include "deadband_flow.h"
#include "foc_pi.h"
#include "motor_current.h"
#include "motor_params_m1.h"
#include "speed_ident_flow.h"
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_module.h"
#endif

#if M1_POS_LOOP_ENABLE && M1_POS_ERR_HYST_ENABLE
static void outer_pos_err_hyst_reset(void);
#endif

#if M1_POS_LOOP_ENABLE && (M1_POS_DECIM > 1u)
static void outer_pos_p_decim_arm(void);
#endif

#if M1_SPEED_LOOP_ENABLE

#if M1_SPEED_PROFILE_ENABLE
static uint32_t s_profile_tick;
static uint8_t s_profile_step;
static uint8_t s_profile_repeat_en;
static volatile uint8_t s_profile_ladder_done;

static float motor_speed_profile_rpm_for_step(uint8_t step)
{
    return M1_SPEED_PROFILE_RPM_START +
           (float)step * M1_SPEED_PROFILE_RPM_STEP;
}

static uint8_t motor_speed_profile_step_count(void)
{
    const float steps_f =
        (M1_SPEED_PROFILE_RPM_END - M1_SPEED_PROFILE_RPM_START) /
        M1_SPEED_PROFILE_RPM_STEP + 1.0f;

    if (steps_f < 1.0f) {
        return 1u;
    }
    if (steps_f > 20.0f) {
        return 20u;
    }
    return (uint8_t)(steps_f + 0.5f);
}

void motor_speed_profile_arm(motor_context_t *ctx)
{
    motor_speed_profile_arm_ex(ctx, (M1_SPEED_PROFILE_REPEAT != 0) ? 1u : 0u);
}

void motor_speed_profile_arm_ex(motor_context_t *ctx, uint8_t repeat_en)
{
    if (ctx == NULL) {
        return;
    }

    s_profile_tick = 0u;
    s_profile_step = 0u;
    s_profile_repeat_en = repeat_en;
    s_profile_ladder_done = 0u;
    ctx->omega_ref = motor_speed_profile_rpm_for_step(0u);
    dbg.outer_omega_ref = ctx->omega_ref;
    dbg.outer_profile_step = 0u;
}

uint8_t motor_speed_profile_consume_ladder_done(void)
{
    if (s_profile_ladder_done == 0u) {
        return 0u;
    }
    s_profile_ladder_done = 0u;
    return 1u;
}

static void motor_speed_profile_tick(motor_context_t *ctx)
{
    const uint32_t hold_ticks =
        (uint32_t)(M1_SPEED_PROFILE_HOLD_S / M1_SPEED_TS_S + 0.5f);
    const uint8_t n_steps = motor_speed_profile_step_count();

    if (hold_ticks == 0u || ctx == NULL) {
        return;
    }

    s_profile_tick++;
    if (s_profile_tick < hold_ticks) {
        return;
    }

    s_profile_tick = 0u;
    if (s_profile_step + 1u < n_steps) {
        s_profile_step++;
    } else if (s_profile_repeat_en != 0u) {
        s_profile_step = 0u;
    } else {
        s_profile_ladder_done = 1u;
        return;
    }

    ctx->omega_ref = motor_speed_profile_rpm_for_step(s_profile_step);
    dbg.outer_omega_ref = ctx->omega_ref;
    dbg.outer_profile_step = s_profile_step;
}
#endif /* M1_SPEED_PROFILE_ENABLE */

#if M1_SPEED_REVERSAL_TEST_ENABLE
static uint8_t s_reversal_armed;
static uint32_t s_reversal_tick;
static int8_t s_reversal_sign;

void motor_speed_reversal_arm(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_reversal_armed = 1u;
    s_reversal_tick = 0u;
    s_reversal_sign = 1;
    ctx->omega_ref = M1_SPEED_REVERSAL_RPM;
    dbg.outer_omega_ref = ctx->omega_ref;
    dbg.outer_profile_step = 0u;
    dbg.open_seq_phase = 201u;
}

static void motor_speed_reversal_tick(motor_context_t *ctx)
{
    const uint32_t hold_ticks =
        (uint32_t)(M1_SPEED_REVERSAL_HOLD_S / M1_SPEED_TS_S + 0.5f);

    if (hold_ticks == 0u || ctx == NULL || s_reversal_armed == 0u) {
        return;
    }

    s_reversal_tick++;
    if (s_reversal_tick < hold_ticks) {
        return;
    }

    s_reversal_tick = 0u;
    if (s_reversal_sign > 0) {
        s_reversal_sign = -1;
        ctx->omega_ref = -M1_SPEED_REVERSAL_RPM;
        dbg.outer_profile_step = 1u;
        dbg.open_seq_phase = 202u;
    } else {
#if M1_SPEED_REVERSAL_REPEAT
        s_reversal_sign = 1;
        ctx->omega_ref = M1_SPEED_REVERSAL_RPM;
        dbg.outer_profile_step = 0u;
        dbg.open_seq_phase = 201u;
#else
        s_reversal_armed = 0u;
        dbg.open_seq_phase = 203u;
#endif
    }
    dbg.outer_omega_ref = ctx->omega_ref;
}
#endif /* M1_SPEED_REVERSAL_TEST_ENABLE */

#if M1_POS_STEP_TEST_ENABLE
static uint8_t s_pos_step_armed;
static uint32_t s_pos_step_tick;
static uint8_t s_pos_step_idx;
static float s_pos_theta0;

/** 相对 θ(0) 的绝对目标；多圈段每档 |Δθ|≤2π，便于 1.5~3 s dwell 内跑完 */
static const float s_pos_theta_offset_rad[] = {
    0.0f,           /* hold */
    0.5235988f,     /* +30° */
    0.7853982f,     /* +45° */
    1.0471976f,     /* +60° */
    1.5707963f,     /* +90° */
    2.0943951f,     /* +120° */
    3.1415927f,     /* +180° */
   -0.7853982f,     /* −45° */
   -1.5707963f,     /* −90° */
   -2.0943951f,     /* −120° */
   -3.1415927f,     /* −180° */
    0.0f,           /* 回零（单圈段结束） */
    6.2831853f,     /* +360°  +1 圈 */
   12.5663706f,     /* +720°  +2 圈 */
   18.8495559f,     /* +1080° +3 圈 */
   25.1327412f,     /* +1440° +4 圈 */
   18.8495559f,     /* +1080° −1 圈 */
   12.5663706f,     /* +720°  −1 圈 */
    6.2831853f,     /* +360°  −1 圈 */
    0.0f,           /* 回零（正向多圈结束） */
   -6.2831853f,     /* −360° −1 圈 */
  -12.5663706f,     /* −720° −2 圈 */
  -18.8495559f,     /* −1080°−3 圈 */
  -12.5663706f,     /*  −720° −2 圈 */
   -6.2831853f,     /*  −360° −1 圈 */
    0.0f,           /* 回零（反向多圈结束） */
};

static uint8_t motor_pos_step_count(void)
{
    return (uint8_t)(sizeof(s_pos_theta_offset_rad) / sizeof(s_pos_theta_offset_rad[0]));
}

static float motor_pos_step_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/** 按本档 |Δθ| 选 dwell：小角 1.5 s，整圈 3 s，≥2 圈 4 s */
static float motor_pos_step_dwell_s(uint8_t step_idx)
{
    float delta_rad;

    if (step_idx == 0u) {
        return M1_POS_STEP_HOLD_S;
    }

    delta_rad = motor_pos_step_fabsf(s_pos_theta_offset_rad[step_idx] -
                                      s_pos_theta_offset_rad[step_idx - 1u]);
    if (delta_rad >= 12.5663706f) {
        return M1_POS_STEP_DWELL_LONG_S;
    }
    if (delta_rad >= 6.2831853f) {
        return M1_POS_STEP_DWELL_MULT_S;
    }
    if (delta_rad >= 3.1415927f) {
        return M1_POS_STEP_DWELL_WIDE_S;
    }
    return M1_POS_STEP_DWELL_S;
}

void motor_pos_step_test_arm(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_pos_theta0 = motor_current_get_theta_mech_rad();
    s_pos_step_idx = 0u;
    s_pos_step_tick = 0u;
    s_pos_step_armed = 1u;
    ctx->theta_ref_rad = s_pos_theta0 + s_pos_theta_offset_rad[0];
#if M1_POS_ERR_HYST_ENABLE
    outer_pos_err_hyst_reset();
#endif
#if M1_POS_DECIM > 1u
    outer_pos_p_decim_arm();
#endif
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
    dbg.outer_profile_step = 0u;
    dbg.open_seq_phase = (uint8_t)M1_POS_STEP_SEQ_BASE;
}

static void motor_pos_step_test_tick(motor_context_t *ctx)
{
    const uint8_t n_steps = motor_pos_step_count();
    const float dwell_s = motor_pos_step_dwell_s(s_pos_step_idx);
    const uint32_t hold_ticks =
        (uint32_t)(dwell_s / M1_SPEED_TS_S + 0.5f);

    if (hold_ticks == 0u || ctx == NULL || s_pos_step_armed == 0u) {
        return;
    }

    s_pos_step_tick++;
    if (s_pos_step_tick < hold_ticks) {
        return;
    }

    s_pos_step_tick = 0u;
    if (s_pos_step_idx + 1u < n_steps) {
        s_pos_step_idx++;
        ctx->theta_ref_rad =
            s_pos_theta0 + s_pos_theta_offset_rad[s_pos_step_idx];
#if M1_POS_DECIM > 1u
        outer_pos_p_decim_arm();
#endif
        dbg.outer_profile_step = s_pos_step_idx;
        dbg.open_seq_phase = (uint8_t)(M1_POS_STEP_SEQ_BASE + s_pos_step_idx);
    } else {
        s_pos_step_armed = 0u;
        dbg.open_seq_phase = (uint8_t)(M1_POS_STEP_SEQ_BASE + n_steps);
    }
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
}
#endif /* M1_POS_STEP_TEST_ENABLE */

#if M1_SPEED_OMEGA_RAMP_ENABLE
static float s_omega_ramped_rpm;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
static float s_iq_ref_slew;
#endif

static float outer_clamp_iq_ref(float ref)
{
    if (ref > M1_SPEED_PI_OUT_MAX) {
        return M1_SPEED_PI_OUT_MAX;
    }
    if (ref < M1_SPEED_PI_OUT_MIN) {
        return M1_SPEED_PI_OUT_MIN;
    }
    return ref;
}

#if M1_SPEED_OMEGA_RAMP_ENABLE
static float outer_omega_ref_ramped(motor_context_t *ctx)
{
    const float target = ctx->omega_ref;
    const float step = M1_SPEED_OMEGA_RAMP_RPM_S * M1_SPEED_TS_S;
    float delta = target - s_omega_ramped_rpm;

    if (delta > step) {
        s_omega_ramped_rpm += step;
    } else if (delta < -step) {
        s_omega_ramped_rpm -= step;
    } else {
        s_omega_ramped_rpm = target;
    }
    return s_omega_ramped_rpm;
}
#endif

static float outer_speed_pi_step(motor_context_t *ctx,
                                  float omega_ref,
                                  float omega_fb,
                                  uint8_t hold_i)
{
    float iq = foc_pi_step_beta_hold_i(&ctx->pi_speed,
                                        omega_ref,
                                        omega_fb,
                                        M1_SPEED_PI_BETA,
                                        hold_i);
#if M1_SPEED_IQ_SLEW_ENABLE
    const float step = M1_SPEED_IQ_SLEW_A_PER_S * M1_SPEED_TS_S;
    float delta = iq - s_iq_ref_slew;

    if (delta > step) {
        s_iq_ref_slew += step;
    } else if (delta < -step) {
        s_iq_ref_slew -= step;
    } else {
        s_iq_ref_slew = iq;
    }
    return outer_clamp_iq_ref(s_iq_ref_slew);
#else
    return iq;
#endif
}

#if M1_POS_LOOP_ENABLE
static float outer_clamp_omega_ref(float rpm)
{
    if (rpm > M1_POS_OMEGA_MAX_RPM) {
        return M1_POS_OMEGA_MAX_RPM;
    }
    if (rpm < -M1_POS_OMEGA_MAX_RPM) {
        return -M1_POS_OMEGA_MAX_RPM;
    }
    return rpm;
}

#if M1_POS_ERR_HYST_ENABLE
static uint8_t s_pos_err_hold_zone;

static float outer_pos_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void outer_pos_err_hyst_reset(void)
{
    s_pos_err_hold_zone = 0u;
}

/** 滞环：|err|<ENTER 进区 → ω_ref=0；在区内 |err|>EXIT 才重新给 P 输出 */
static float outer_pos_omega_from_err(float err_rad)
{
    const float aerr = outer_pos_fabsf(err_rad);

    if (s_pos_err_hold_zone) {
        if (aerr > M1_POS_ERR_HYST_EXIT_RAD) {
            s_pos_err_hold_zone = 0u;
        }
    } else if (aerr < M1_POS_ERR_HYST_ENTER_RAD) {
        s_pos_err_hold_zone = 1u;
    }

    if (s_pos_err_hold_zone) {
        return 0.0f;
    }
    return outer_clamp_omega_ref(M1_POS_KP_RPM_PER_RAD * err_rad);
}
#endif /* M1_POS_ERR_HYST_ENABLE */

#if M1_POS_DECIM > 1u
static uint8_t s_pos_p_div;

static void outer_pos_p_decim_arm(void)
{
    s_pos_p_div = M1_POS_DECIM;
}
#endif /* M1_POS_DECIM > 1u */
#endif /* M1_POS_LOOP_ENABLE */

void motor_outer_loop_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    foc_pi_init(&ctx->pi_speed,
                M1_SPEED_PI_KP,
                M1_SPEED_PI_KI,
                M1_SPEED_PI_OUT_MIN,
                M1_SPEED_PI_OUT_MAX,
                M1_SPEED_PI_INT_MIN,
                M1_SPEED_PI_INT_MAX);

    ctx->outer_mode = M1_OUTER_DISABLED;
    ctx->omega_ref  = 0.0f;
    ctx->theta_ref_rad = 0.0f;
    ctx->iq_cmd     = 0.0f;
#if M1_POS_LOOP_ENABLE && M1_POS_ERR_HYST_ENABLE
    outer_pos_err_hyst_reset();
#endif
#if M1_POS_LOOP_ENABLE && (M1_POS_DECIM > 1u)
    outer_pos_p_decim_arm();
#endif
#if M1_SPEED_OMEGA_RAMP_ENABLE
    s_omega_ramped_rpm = 0.0f;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
    s_iq_ref_slew = 0.0f;
#endif
}

void motor_outer_loop_tick(motor_context_t *ctx)
{
    float omega_mech_rpm;
    float theta_mech_rad;

    if (ctx == NULL) {
        return;
    }

#if M1_ID_LOCK_CAL_SWEEP
    /* Pass0 / VASI：外环不得写 iq_ref/ω_ref（旧版 speed 联调曾导致转子旋转） */
    if (deadband_flow_id_cal_active()) {
        return;
    }
#endif

#if M1_PLL_ENABLE
    omega_mech_rpm = motor_current_get_pll_omega_mech_rpm();
#else
    omega_mech_rpm = 0.0f;
#endif
    theta_mech_rad = motor_current_get_theta_mech_rad();

    dbg.outer_omega_mech_rpm = omega_mech_rpm;
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
    dbg.outer_theta_err_rad = ctx->theta_ref_rad - theta_mech_rad;

    switch (ctx->outer_mode) {
    case M1_OUTER_TORQUE:
        ctx->iq_ref = outer_clamp_iq_ref(ctx->iq_cmd);
        ctx->id_ref = 0.0f;
        break;

#if M1_POS_LOOP_ENABLE
    case M1_OUTER_POSITION:
#if M1_POS_STEP_TEST_ENABLE
        if (s_pos_step_armed) {
            motor_pos_step_test_tick(ctx);
        }
#endif
        {
            uint8_t pos_p_tick = 0u;

#if M1_POS_DECIM > 1u
            s_pos_p_div++;
            if (s_pos_p_div >= M1_POS_DECIM) {
                s_pos_p_div = 0u;
                pos_p_tick = 1u;
            }
#else
            pos_p_tick = 1u;
#endif

            if (pos_p_tick != 0u) {
                const float err_rad = ctx->theta_ref_rad - theta_mech_rad;

#if M1_POS_ERR_HYST_ENABLE
                const uint8_t was_hold = s_pos_err_hold_zone;

                ctx->omega_ref = outer_pos_omega_from_err(err_rad);
#if M1_POS_ERR_HYST_FREEZE_SPEED_PI
                if (s_pos_err_hold_zone) {
                    if (was_hold == 0u) {
                        foc_pi_reset(&ctx->pi_speed);
                    }
                } else if (was_hold != 0u) {
                    foc_pi_bumpless_beta(&ctx->pi_speed, 0.0f,
                                         ctx->omega_ref, omega_mech_rpm,
                                         M1_SPEED_PI_BETA);
                }
#else
                (void)was_hold;
#endif
#else
                ctx->omega_ref =
                    outer_clamp_omega_ref(M1_POS_KP_RPM_PER_RAD * err_rad);
#endif /* M1_POS_ERR_HYST_ENABLE */
            }

#if M1_POS_ERR_HYST_ENABLE && M1_POS_ERR_HYST_FREEZE_SPEED_PI
            if (s_pos_err_hold_zone) {
                ctx->iq_ref = 0.0f;
            } else
#endif
            {
#if M1_SPEED_OMEGA_RAMP_ENABLE
                ctx->iq_ref = outer_speed_pi_step(ctx,
                                                   outer_omega_ref_ramped(ctx),
                                                   omega_mech_rpm,
                                                   0u);
#else
                ctx->iq_ref = outer_speed_pi_step(ctx,
                                                   ctx->omega_ref,
                                                   omega_mech_rpm,
                                                   0u);
#endif
            }
        }
        ctx->id_ref = 0.0f;
        break;
#endif

    case M1_OUTER_SPEED:
        {
            uint8_t speed_hold_i = 0u;

#if M1_SPEED_IDENT_ENABLE
            if (speed_ident_flow_is_armed()) {
                speed_ident_flow_tick(ctx);
            } else
#endif
            {
#if M1_SPEED_REVERSAL_TEST_ENABLE
                if (s_reversal_armed) {
                    motor_speed_reversal_tick(ctx);
                }
#endif
#if M1_SPEED_PROFILE_ENABLE
#if M1_SPEED_REVERSAL_TEST_ENABLE
                else if (!s_reversal_armed)
#else
                if (1)
#endif
                {
                    motor_speed_profile_tick(ctx);
                }
#endif
            }
#if M1_SPEED_IDENT_ENABLE
            if (speed_ident_flow_is_armed() &&
                speed_ident_module_hold_iq_inhibit()) {
                speed_hold_i = 1u;
            }
#endif
#if M1_SPEED_OMEGA_RAMP_ENABLE
            ctx->iq_ref = outer_speed_pi_step(ctx,
                                               outer_omega_ref_ramped(ctx),
                                               omega_mech_rpm,
                                               speed_hold_i);
#else
            ctx->iq_ref = outer_speed_pi_step(ctx,
                                               ctx->omega_ref,
                                               omega_mech_rpm,
                                               speed_hold_i);
#endif
            if (speed_hold_i != 0u) {
                ctx->iq_ref = 0.0f;
            }
            ctx->id_ref = 0.0f;
        }
        break;

    case M1_OUTER_DISABLED:
    default:
        /* 不写 iq_ref：保持 startup / 标定 / API 写入的值 */
        break;
    }

    dbg.outer_iq_ref = ctx->iq_ref;
    dbg.outer_omega_ref = ctx->omega_ref;
}

void motor_outer_arm_position_hold(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->theta_ref_rad = motor_current_get_theta_mech_rad();
    ctx->omega_ref = 0.0f;
#if M1_POS_ERR_HYST_ENABLE
    outer_pos_err_hyst_reset();
#endif
#if M1_POS_DECIM > 1u
    outer_pos_p_decim_arm();
#endif
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
    dbg.outer_omega_ref = 0.0f;
}

void motor_outer_sync_speed_boot(motor_context_t *ctx,
                                  float iq_meas,
                                  float omega_now)
{
    if (ctx == NULL) {
        return;
    }

#if M1_SPEED_OMEGA_RAMP_ENABLE
    s_omega_ramped_rpm = omega_now;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
    s_iq_ref_slew = iq_meas;
#endif
    if (ctx->outer_mode == M1_OUTER_SPEED) {
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas,
                              ctx->omega_ref, omega_now,
                              M1_SPEED_PI_BETA);
    }
}

void motor_outer_set_mode(motor_context_t *ctx,
                           m1_outer_mode_t new_mode,
                           float iq_meas,
                           float omega_now)
{
    if (ctx == NULL) {
        return;
    }

    if (new_mode == ctx->outer_mode) {
        return;
    }

    switch (new_mode) {
    case M1_OUTER_SPEED:
        /* TORQUE/DISABLED/POSITION → SPEED：用当前 iq_meas 做 bumpless */
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas,
                              ctx->omega_ref, omega_now,
                              M1_SPEED_PI_BETA);
#if M1_SPEED_OMEGA_RAMP_ENABLE
        s_omega_ramped_rpm = omega_now;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
        s_iq_ref_slew = iq_meas;
#endif
        break;

#if M1_POS_LOOP_ENABLE
    case M1_OUTER_POSITION:
        motor_outer_arm_position_hold(ctx);
#if M1_POS_ERR_HYST_ENABLE
        outer_pos_err_hyst_reset();
#endif
#if M1_POS_DECIM > 1u
        outer_pos_p_decim_arm();
#endif
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas, 0.0f, omega_now,
                              M1_SPEED_PI_BETA);
#if M1_SPEED_OMEGA_RAMP_ENABLE
        s_omega_ramped_rpm = omega_now;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
        s_iq_ref_slew = iq_meas;
#endif
        break;
#endif

    case M1_OUTER_TORQUE:
        /* SPEED → TORQUE：重置速度 PI，防止下次切回时积分残留 */
        foc_pi_reset(&ctx->pi_speed);
        break;

    case M1_OUTER_DISABLED:
    default:
        /* 切回 DISABLED 时重置，重新进入 startup/标定不残留速度 PI 积分 */
        foc_pi_reset(&ctx->pi_speed);
        break;
    }

    ctx->outer_mode = new_mode;
    dbg.outer_mode = (uint8_t)new_mode;
}

#else /* M1_SPEED_LOOP_ENABLE */

void motor_outer_loop_init(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_outer_loop_tick(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_outer_set_mode(motor_context_t *ctx,
                           m1_outer_mode_t new_mode,
                           float iq_meas,
                           float omega_now)
{
    (void)ctx;
    (void)new_mode;
    (void)iq_meas;
    (void)omega_now;
}

void motor_outer_arm_position_hold(motor_context_t *ctx)
{
    (void)ctx;
}

#endif /* M1_SPEED_LOOP_ENABLE */
